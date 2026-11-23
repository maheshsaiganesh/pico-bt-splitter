// Step 3: iPhone -> Pico W -> both UE Booms.
//
// The Pico is an A2DP sink for the iPhone ("Pico Car Audio") and an A2DP
// source for each Boom. Everyone is pinned to the same SBC settings (44.1 kHz,
// joint stereo, 16 blocks, 8 subbands, loudness, bitpool <= 53), so the
// iPhone's SBC frames are forwarded to the Booms as-is: no decode/re-encode,
// no quality loss, very little CPU.
//
// Step 4: the phone's audio is also SBC-decoded and played on the AUX wires
// (PWM on GP14/GP15, see pwm_audio.c), held back by a saved delay so the car
// lines up with the Booms. The AUX playback speed is trimmed a few hundred
// ppm to follow the phone's clock, so the delay stays put.
//
// Booms are remembered in flash (same slots as boom_test). When the iPhone
// starts playing, all Booms are paused and restarted together so they stay in
// sync; when it pauses, the Booms pause too.
//
// Serial commands: s = status, r = resync Booms, d = disconnect all,
// f = forget both Booms (they'll be scanned for again),
// + / - = AUX delay up / down 10 ms (saved).
//
// BOOTSEL button (for the car, no laptop): 1 press = AUX 10 ms later,
// 2 quick presses = AUX 10 ms earlier, hold 2 s = resync the Booms.
// The LED blinks once / twice / three times to confirm.
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "btstack_tlv.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pwm_audio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#define SMPS_MODE_PIN 1 // CYW43 WL_GPIO1: regulator PWM mode (less ripple)
#define SBC_STORAGE_SIZE 1030
#define QUEUE_PACKETS 8 // per Boom: ~8 phone packets of slack (~100+ ms)
#define INQUIRY_DURATION 8
#define RETRY_MS 3000
#define RETRY_WHILE_PLAYING_MS 15000
#define PAGE_TIMEOUT_SLOTS 3200 // 2 s
#define MAX_BOOMS 2
#define SAMPLES_PER_FRAME 128 // 16 blocks x 8 subbands
// AUX output (step 1 settings: 488 kHz, right channel antiphase, 1st-order
// shaping, ~3 V p-p, no silence gate).
#define PIN_AUX_LEFT 14 // right = GP15
#define AUX_GAIN_PERCENT 90
#define AUX_SHAPING 1
#define AUX_DEFAULT_DELAY_MS 180 // calibrated by ear at home (snares) against a wired speaker
#define AUX_MAX_DELAY_MS 340 // the PWM ring holds ~371 ms
#define TLV_TAG_AUX_DELAY ((uint32_t)'A' << 24 | 'U' << 16 | 'X' << 8 | 'D')
#define TLV_TAG_OLD ((uint32_t)'B' << 24 | 'O' << 16 | 'O' << 8 | 'M')
#define TLV_TAG_BOOM(i) ((uint32_t)'B' << 24 | 'O' << 16 | 'M' << 8 | ('1' + (i)))

// One fixed SBC configuration for every link, so frames can be forwarded.
#define SBC_CAPS                                                                           \
    {(AVDTP_SBC_44100 << 4) | AVDTP_SBC_JOINT_STEREO,                                      \
     (AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_LOUDNESS, \
     2, 53}
static uint8_t source_caps[MAX_BOOMS][4] = {SBC_CAPS, SBC_CAPS};
static uint8_t source_config[MAX_BOOMS][4];
// The phone's bitpool sets the bitrate on all three links. 35 is SBC's
// standard "middle quality" for 44.1 kHz joint stereo (~230 kbit/s vs ~330 at
// 53); three links at 53 was more than the Pico's radio could carry.
#define PHONE_MAX_BITPOOL 53
static uint8_t sink_caps[4] = SBC_CAPS;
static uint8_t sink_config[4];

// --- Booms (A2DP sinks we send to) ------------------------------------------
typedef enum { B_EMPTY, B_IDLE, B_CONNECTING, B_CONNECTED, B_STREAMING } boom_state_t;
static const char *boom_state_names[] = {"empty", "idle", "connecting", "connected", "streaming"};

typedef struct {
    uint16_t len;       // bytes of SBC frames
    uint8_t num_frames;
    uint8_t data[SBC_STORAGE_SIZE]; // data[0] = SBC media header
} packet_t;

typedef struct {
    boom_state_t state;
    bd_addr_t addr;
    uint16_t a2dp_cid;
    uint8_t local_seid;
    int max_payload;
    uint32_t rtp_timestamp;
    packet_t q[QUEUE_PACKETS];
    uint8_t q_head, q_count;
    bool tail_open;     // the newest packet can still take frames
    bool send_pending;  // waiting for CAN_SEND_MEDIA_PACKET_NOW
    uint32_t packets_sent, frames_dropped;
    uint32_t last_send_ms, max_gap_ms; // longest pause between packets (per report)
    uint8_t max_queue;
} boom_t;
static boom_t booms[MAX_BOOMS];

// --- iPhone (A2DP source that sends to us) ----------------------------------
static struct {
    bool connected, streaming;
    uint16_t a2dp_cid;
    uint8_t local_seid;
    bd_addr_t addr;
    uint32_t packets_in, frames_in, bad_packets;
    uint32_t last_in_ms, max_gap_ms;
} phone;

// --- AUX output state ------------------------------------------------------
static const btstack_sbc_decoder_t *sbc_dec;
static btstack_sbc_decoder_bluedroid_t sbc_dec_state;
static uint32_t aux_delay_ms = AUX_DEFAULT_DELAY_MS;
static uint64_t fill_sum;        // queue depth right after each phone packet
static uint32_t fill_count;
static int32_t trim_ppm;
static uint32_t aux_dropped;     // frames that didn't fit in the PWM ring
static uint32_t aux_skip;        // decoded frames to discard (to cut the delay)
static bool aux_need_prime;      // top up at the first packet of a stream
static uint32_t aux_jumps;       // big corrections made
static uint32_t rate_t0_ms, rate_in0, rate_out0; // stream start, for measured in/out rates
static uint64_t decode_us;       // time spent decoding since last report
static uint32_t report_start_ms;

static bool resyncing;
static uint32_t resync_started_ms, last_attempt_ms;
static bool scanning;
static btstack_timer_source_t manager_timer, stats_timer;
static btstack_packet_callback_registration_t hci_callback;
static uint8_t sdp_source_buffer[150], sdp_sink_buffer[150];

// --- flash -------------------------------------------------------------------
static void tlv(const btstack_tlv_t **impl, void **ctx) { btstack_tlv_get_instance(impl, ctx); }

static void load_booms(void) {
    const btstack_tlv_t *t;
    void *c;
    tlv(&t, &c);
    for (int i = 0; i < MAX_BOOMS; i++)
        if (t->get_tag(c, TLV_TAG_BOOM(i), booms[i].addr, sizeof(bd_addr_t)) == sizeof(bd_addr_t))
            booms[i].state = B_IDLE;
    bd_addr_t old;
    if (booms[0].state == B_EMPTY && t->get_tag(c, TLV_TAG_OLD, old, sizeof old) == sizeof old) {
        bd_addr_copy(booms[0].addr, old);
        booms[0].state = B_IDLE;
        t->store_tag(c, TLV_TAG_BOOM(0), old, sizeof old);
        t->delete_tag(c, TLV_TAG_OLD);
    }
    for (int i = 0; i < MAX_BOOMS; i++)
        if (booms[i].state != B_EMPTY) printf("Boom %d remembered: %s\n", i + 1, bd_addr_to_str(booms[i].addr));
}

static void save_boom(int i) {
    const btstack_tlv_t *t;
    void *c;
    tlv(&t, &c);
    bd_addr_t stored;
    if (t->get_tag(c, TLV_TAG_BOOM(i), stored, sizeof stored) == sizeof stored &&
        bd_addr_cmp(stored, booms[i].addr) == 0)
        return;
    t->store_tag(c, TLV_TAG_BOOM(i), booms[i].addr, sizeof(bd_addr_t));
    printf("Saved Boom %d (%s).\n", i + 1, bd_addr_to_str(booms[i].addr));
}

static void forget_all(void) {
    const btstack_tlv_t *t;
    void *c;
    tlv(&t, &c);
    for (int i = 0; i < MAX_BOOMS; i++) {
        t->delete_tag(c, TLV_TAG_BOOM(i));
        if (booms[i].state != B_EMPTY) gap_drop_link_key_for_bd_addr(booms[i].addr);
        if (booms[i].a2dp_cid) a2dp_source_disconnect(booms[i].a2dp_cid);
        memset(&booms[i], 0, sizeof booms[i]);
    }
    printf("Forgot both Booms.\n");
}

static int boom_for_cid(uint16_t cid) {
    for (int i = 0; i < MAX_BOOMS; i++)
        if (booms[i].state >= B_CONNECTING && booms[i].a2dp_cid == cid) return i;
    return -1;
}

static int boom_for_addr(const bd_addr_t a) {
    for (int i = 0; i < MAX_BOOMS; i++)
        if (booms[i].state != B_EMPTY && bd_addr_cmp(booms[i].addr, a) == 0) return i;
    return -1;
}

static int known_booms(void) {
    int n = 0;
    for (int i = 0; i < MAX_BOOMS; i++) n += booms[i].state != B_EMPTY;
    return n;
}

// --- forwarding --------------------------------------------------------------
static void boom_reset_queue(boom_t *b) {
    b->q_head = b->q_count = 0;
    b->tail_open = false;
}

static void boom_add_frame(boom_t *b, const uint8_t *frame, int len) {
    packet_t *tail = b->q_count ? &b->q[(b->q_head + b->q_count - 1) % QUEUE_PACKETS] : NULL;
    if (!tail || !b->tail_open || tail->len + len > b->max_payload) {
        if (b->q_count == QUEUE_PACKETS) { b->frames_dropped++; return; } // link backed up
        tail = &b->q[(b->q_head + b->q_count) % QUEUE_PACKETS];
        tail->len = 0;
        tail->num_frames = 0;
        b->q_count++;
        b->tail_open = true;
    }
    memcpy(&tail->data[1 + tail->len], frame, len);
    tail->len += len;
    tail->num_frames++;
}

// Packets are filled to the Boom's maximum size before they're sent: fewer,
// fuller packets use the radio more efficiently.
static int complete_packets(const boom_t *b) { return b->q_count - (b->tail_open ? 1 : 0); }

static void boom_kick(boom_t *b) {
    if (b->state == B_STREAMING && complete_packets(b) > 0 && !b->send_pending) {
        b->send_pending = true;
        a2dp_source_stream_endpoint_request_can_send_now(b->a2dp_cid, b->local_seid);
    }
}

static void boom_send(boom_t *b) {
    b->send_pending = false;
    if (b->state != B_STREAMING || complete_packets(b) <= 0) return;
    packet_t *p = &b->q[b->q_head];
    p->data[0] = p->num_frames;                // SBC media header
    a2dp_source_stream_send_media_payload_rtp(b->a2dp_cid, b->local_seid, 0, b->rtp_timestamp, p->data, p->len + 1);
    b->rtp_timestamp += p->num_frames * SAMPLES_PER_FRAME;
    b->q_head = (b->q_head + 1) % QUEUE_PACKETS;
    b->q_count--;
    b->packets_sent++;
    uint32_t now = btstack_run_loop_get_time_ms();
    if (b->last_send_ms && now - b->last_send_ms > b->max_gap_ms) b->max_gap_ms = now - b->last_send_ms;
    b->last_send_ms = now;
    boom_kick(b);
}

// SBC frame length from its 4-byte header (A2DP spec, 12.9).
static int sbc_frame_len(const uint8_t *f, int avail) {
    if (avail < 4 || f[0] != 0x9c) return -1;
    int blocks = 4 * (((f[1] >> 4) & 3) + 1);
    int mode = (f[1] >> 2) & 3; // 0 mono, 1 dual, 2 stereo, 3 joint
    int subbands = (f[1] & 1) ? 8 : 4;
    int bitpool = f[2];
    int channels = mode == 0 ? 1 : 2;
    int len = 4 + (4 * subbands * channels) / 8;
    if (mode <= 1) len += (blocks * channels * bitpool + 7) / 8;
    else len += ((mode == 3 ? subbands : 0) + blocks * bitpool + 7) / 8;
    return len <= avail ? len : -1;
}

// --- AUX ----------------------------------------------------------------------
static uint32_t aux_delay_frames(void) { return aux_delay_ms * 441 / 10; }

static void aux_pcm(int16_t *data, int num_frames, int num_channels, int sample_rate, void *ctx) {
    (void)sample_rate;
    (void)ctx;
    for (int i = 0; i < num_frames; i++) {
        if (aux_skip) { aux_skip--; continue; }
        int16_t l = data[i * num_channels], r = num_channels > 1 ? data[i * num_channels + 1] : l;
        if (!pwm_audio_push(l, r)) aux_dropped++;
    }
}

// Top the queue up with silence to the target delay (stream start, or after
// the delay was raised). Lowering the delay is left to the speed trim.
static void aux_fill_to(uint32_t target) {
    for (uint32_t q = pwm_audio_queued_frames(); q < target; q++) pwm_audio_push(0, 0);
}
static void aux_prime(void) { aux_fill_to(aux_delay_frames()); }

// Once a second: steer playback speed so the queue sits at the target delay.
// PI control. P: 10 ms off -> 300 ppm. I: +1 ppm per second per 10 ms off,
// capped at +/-300 ppm, learns the real phone-vs-Pico clock difference so the
// delay settles exactly on target. The phone sends a little slow for the
// first ~20 s after play (measured 43,970 vs 44,100 Hz), which is not a
// clock difference, so the I term doesn't learn in the first 10 s. Both capped at 1000 ppm. The learned clock
// offset is kept across pauses.
static int32_t trim_integral_ppm;

static void aux_steer(void) {
    if (!phone.streaming || !fill_count) { pwm_audio_set_rate_trim_ppm(trim_integral_ppm); return; }
    int32_t avg = (int32_t)(fill_sum / fill_count);
    fill_sum = 0;
    fill_count = 0;
    int32_t err = avg - (int32_t)aux_delay_frames();
    // More than 20 ms off: jump straight to target (silence in, or skip out).
    if (err > 882 || err < -882) {
        if (err > 0) aux_skip += (uint32_t)err;
        else aux_fill_to(pwm_audio_queued_frames() + (uint32_t)-err);
        aux_jumps++;
        return;
    }
    if (btstack_run_loop_get_time_ms() - rate_t0_ms > 10000) trim_integral_ppm += err / 441;
    if (trim_integral_ppm > 300) trim_integral_ppm = 300;
    if (trim_integral_ppm < -300) trim_integral_ppm = -300;
    trim_ppm = err * 300 / 441 + trim_integral_ppm;
    if (trim_ppm > 1000) trim_ppm = 1000;
    if (trim_ppm < -1000) trim_ppm = -1000;
    pwm_audio_set_rate_trim_ppm(trim_ppm);
}

static void load_aux_delay(void) {
    const btstack_tlv_t *t;
    void *c;
    tlv(&t, &c);
    uint32_t v;
    if (t->get_tag(c, TLV_TAG_AUX_DELAY, (uint8_t *)&v, sizeof v) == sizeof v && v <= AUX_MAX_DELAY_MS)
        aux_delay_ms = v;
    printf("AUX delay %lu ms\n", (unsigned long)aux_delay_ms);
}

static void set_aux_delay(int32_t ms) {
    if (ms < 0) ms = 0;
    if (ms > AUX_MAX_DELAY_MS) ms = AUX_MAX_DELAY_MS;
    aux_delay_ms = (uint32_t)ms;
    if (phone.streaming) aux_prime(); // raising takes effect now; lowering via the jump/skip
    const btstack_tlv_t *t;
    void *c;
    tlv(&t, &c);
    t->store_tag(c, TLV_TAG_AUX_DELAY, (const uint8_t *)&aux_delay_ms, sizeof aux_delay_ms);
    printf("AUX delay now %lu ms (saved)\n", (unsigned long)aux_delay_ms);
}

static void phone_media_handler(uint8_t seid, uint8_t *packet, uint16_t size) {
    (void)seid;

    if (size < 13) { phone.bad_packets++; return; }
    int pos = 12 + 4 * (packet[0] & 0x0f); // RTP header + CSRCs (count = low 4 bits)
    if (pos >= size) { phone.bad_packets++; return; }
    uint8_t sbc_header = packet[pos++];
    if (sbc_header & 0x80) { phone.bad_packets++; return; } // fragmented: not expected for SBC
    int num_frames = sbc_header & 0x0f;
    phone.packets_in++;
    uint32_t now = btstack_run_loop_get_time_ms();
    if (phone.last_in_ms && now - phone.last_in_ms > phone.max_gap_ms) phone.max_gap_ms = now - phone.last_in_ms;
    phone.last_in_ms = now;
    // AUX: decode every packet (even while the Booms resync), then note how
    // full the queue is right after it, for the speed trim.
    if (aux_need_prime) {
        // First audio of this stream: silence so that, once this packet is
        // decoded, exactly the target delay is queued.
        uint32_t target = aux_delay_frames(), pkt = (uint32_t)num_frames * SAMPLES_PER_FRAME;
        aux_fill_to(target > pkt ? target - pkt : 0);
        aux_skip = 0;
        aux_need_prime = false;
    }
    uint32_t t0 = time_us_32();
    sbc_dec->decode_signed_16(&sbc_dec_state, 0, packet + pos, size - pos);
    decode_us += time_us_32() - t0;
    fill_sum += pwm_audio_queued_frames();
    fill_count++;

    if (resyncing) return; // Booms are restarting; drop until they're all back

    for (int n = 0; n < num_frames; n++) {
        int len = sbc_frame_len(packet + pos, size - pos);
        if (len < 0) { phone.bad_packets++; break; }
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].state == B_STREAMING) boom_add_frame(&booms[i], packet + pos, len);
        pos += len;
        phone.frames_in++;
    }
    for (int i = 0; i < MAX_BOOMS; i++) {
        if (booms[i].q_count > booms[i].max_queue) booms[i].max_queue = booms[i].q_count;
        boom_kick(&booms[i]);
    }
}

// --- start / pause / resync the Booms ------------------------------------
static void start_boom(boom_t *b) {
    boom_reset_queue(b);
    b->last_send_ms = 0;
    a2dp_source_start_stream(b->a2dp_cid, b->local_seid);
}

static void pause_booms(void) {
    for (int i = 0; i < MAX_BOOMS; i++) {
        boom_t *b = &booms[i];
        if (b->state != B_STREAMING) continue;
        b->state = B_CONNECTED;
        boom_reset_queue(b);
        a2dp_source_pause_stream(b->a2dp_cid, b->local_seid);
    }
}

static void resync_step(void) {
    if (!resyncing) return;
    bool any_streaming = false, any_connected = false;
    for (int i = 0; i < MAX_BOOMS; i++) {
        any_streaming |= booms[i].state == B_STREAMING;
        any_connected |= booms[i].state == B_CONNECTED;
    }
    if (any_connected && !any_streaming) {
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].state == B_CONNECTED) start_boom(&booms[i]);
        return;
    }
    if (any_streaming && !any_connected) {
        resyncing = false;
        printf("Booms started together.\n");
    } else if (!any_streaming && !any_connected) {
        resyncing = false; // no Booms around
    }
}

// Pause every Boom and restart them all at once, so each starts from the same
// frame with an empty buffer.
static void resync(void) {
    if (!phone.streaming) return;
    pause_booms();
    resyncing = true;
    resync_started_ms = btstack_run_loop_get_time_ms();
    resync_step();
}

// --- connection manager ------------------------------------------------------
static void start_scan(void) {
    if (scanning) return;
    printf("Scanning for a new Boom (put it in pairing mode)...\n");
    scanning = true;
    gap_inquiry_start(INQUIRY_DURATION);
}

static void connect_boom(int i) {
    boom_t *b = &booms[i];
    printf("Connecting to Boom %d (%s)...\n", i + 1, bd_addr_to_str(b->addr));
    b->state = B_CONNECTING;
    uint8_t status = a2dp_source_establish_stream(b->addr, &b->a2dp_cid);
    if (status != ERROR_CODE_SUCCESS) {
        printf("  could not start (0x%02x)\n", status);
        b->state = B_IDLE;
    }
}

static void manager(btstack_timer_source_t *t) {
    btstack_run_loop_set_timer(t, 1000);
    btstack_run_loop_add_timer(t);
    aux_steer();
    uint32_t now = btstack_run_loop_get_time_ms();
    if (resyncing && now - resync_started_ms > 3000) {
        printf("Resync timed out; starting whoever is ready.\n");
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].state == B_CONNECTED && phone.streaming) start_boom(&booms[i]);
        resyncing = false;
    }
    if (scanning) return;
    bool playing = phone.streaming;
    for (int i = 0; i < MAX_BOOMS; i++)
        if (booms[i].state == B_CONNECTING) return;
    if (now - last_attempt_ms < (playing ? RETRY_WHILE_PLAYING_MS : RETRY_MS)) return;
    for (int i = 0; i < MAX_BOOMS; i++)
        if (booms[i].state == B_IDLE) { last_attempt_ms = now; connect_boom(i); return; }
    if (known_booms() < MAX_BOOMS && !playing) { last_attempt_ms = now; start_scan(); }
}

static bool is_boom_name(const char *name) {
    for (const char *p = name; p[0] && p[1] && p[2] && p[3]; p++)
        if ((p[0] | 32) == 'b' && (p[1] | 32) == 'o' && (p[2] | 32) == 'o' && (p[3] | 32) == 'm')
            return true;
    return false;
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    bd_addr_t addr;

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
        gap_local_bd_addr(addr);
        printf("Bluetooth up. Pair your phone with 'Pico Car Audio' (%s).\n", bd_addr_to_str(addr));
        load_booms();
        load_aux_delay();
        btstack_run_loop_set_timer_handler(&manager_timer, manager);
        btstack_run_loop_set_timer(&manager_timer, 100);
        btstack_run_loop_add_timer(&manager_timer);
        break;

    case HCI_EVENT_PIN_CODE_REQUEST:
        hci_event_pin_code_request_get_bd_addr(packet, addr);
        gap_pin_code_response(addr, "0000");
        break;

    case HCI_EVENT_USER_CONFIRMATION_REQUEST:
        hci_event_user_confirmation_request_get_bd_addr(packet, addr);
        printf("Pairing with %s\n", bd_addr_to_str(addr));
        gap_ssp_confirmation_response(addr);
        break;

    case GAP_EVENT_INQUIRY_RESULT: {
        if (!scanning) break;
        gap_event_inquiry_result_get_bd_addr(packet, addr);
        char name[64] = "";
        if (gap_event_inquiry_result_get_name_available(packet)) {
            int len = gap_event_inquiry_result_get_name_len(packet);
            if (len > (int)sizeof name - 1) len = sizeof name - 1;
            memcpy(name, gap_event_inquiry_result_get_name(packet), len);
            name[len] = 0;
        }
        if (!name[0] || !is_boom_name(name) || boom_for_addr(addr) >= 0) break;
        for (int i = 0; i < MAX_BOOMS; i++) {
            if (booms[i].state != B_EMPTY) continue;
            printf("Found '%s' at %s -> Boom %d\n", name, bd_addr_to_str(addr), i + 1);
            bd_addr_copy(booms[i].addr, addr);
            booms[i].state = B_IDLE;
            gap_inquiry_stop();
            scanning = false;
            connect_boom(i);
            break;
        }
        break;
    }

    case GAP_EVENT_INQUIRY_COMPLETE:
        scanning = false;
        break;

    default:
        break;
    }
}

// Events for our links to the Booms.
static void source_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;
    uint8_t status;
    bd_addr_t addr;
    int i;

    switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
        a2dp_subevent_signaling_connection_established_get_bd_addr(packet, addr);
        i = boom_for_addr(addr);
        if (i < 0) break;
        status = a2dp_subevent_signaling_connection_established_get_status(packet);
        if (status != ERROR_CODE_SUCCESS) {
            printf("Boom %d: connection failed (0x%02x).\n", i + 1, status);
            booms[i].state = B_IDLE;
            booms[i].a2dp_cid = 0;
            break;
        }
        booms[i].a2dp_cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
        booms[i].state = B_CONNECTED;
        printf("Boom %d: connected.\n", i + 1);
        break;

    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:
        printf("Boom SBC: %d Hz, mode %d, %d blocks, %d subbands, bitpool %d-%d\n",
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet));
        break;

    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
        a2dp_subevent_stream_established_get_bd_addr(packet, addr);
        i = boom_for_addr(addr);
        if (i < 0) break;
        status = a2dp_subevent_stream_established_get_status(packet);
        if (status != ERROR_CODE_SUCCESS) {
            printf("Boom %d: stream setup failed (0x%02x)\n", i + 1, status);
            break;
        }
        booms[i].a2dp_cid = a2dp_subevent_stream_established_get_a2dp_cid(packet);
        booms[i].local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
        booms[i].max_payload = btstack_min(a2dp_max_media_payload_size(booms[i].a2dp_cid, booms[i].local_seid),
                                           SBC_STORAGE_SIZE - 1);
        booms[i].state = B_CONNECTED;
        save_boom(i);
        printf("Boom %d: ready, max packet payload %d bytes.\n", i + 1, booms[i].max_payload);
        if (phone.streaming) resync(); // join the music in step with the other Boom
        break;

    case A2DP_SUBEVENT_STREAM_STARTED:
        i = boom_for_cid(a2dp_subevent_stream_started_get_a2dp_cid(packet));
        if (i < 0) break;
        booms[i].state = B_STREAMING;
        booms[i].send_pending = false;
        boom_reset_queue(&booms[i]);
        printf("Boom %d: playing.\n", i + 1);
        resync_step();
        break;

    case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
        i = boom_for_cid(a2dp_subevent_streaming_can_send_media_packet_now_get_a2dp_cid(packet));
        if (i >= 0) boom_send(&booms[i]);
        break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:
        i = boom_for_cid(a2dp_subevent_stream_suspended_get_a2dp_cid(packet));
        if (i < 0) break;
        booms[i].state = B_CONNECTED;
        resync_step();
        break;

    case A2DP_SUBEVENT_STREAM_RELEASED:
        i = boom_for_cid(a2dp_subevent_stream_released_get_a2dp_cid(packet));
        if (i >= 0 && booms[i].state == B_STREAMING) booms[i].state = B_CONNECTED;
        break;

    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        i = boom_for_cid(a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet));
        if (i < 0) break;
        printf("Boom %d: disconnected, will reconnect.\n", i + 1);
        booms[i].state = B_IDLE;
        booms[i].a2dp_cid = 0;
        resync_step();
        break;

    default:
        break;
    }
}

// Events for the iPhone's link to us.
static void sink_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;
    uint8_t sub = hci_event_a2dp_meta_get_subevent_code(packet);
    // Signaling events reach both the sink and source handlers; skip Booms'.
    // A2DP meta events carry the a2dp cid right after the subevent code.
    uint16_t cid = little_endian_read_16(packet, 3);
    if (sub == A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED) {
        bd_addr_t addr;
        a2dp_subevent_signaling_connection_established_get_bd_addr(packet, addr);
        if (boom_for_addr(addr) >= 0) return;
    } else if (!phone.connected || cid != phone.a2dp_cid) {
        if (sub != A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION || boom_for_cid(cid) >= 0) return;
    }

    switch (sub) {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
        if (a2dp_subevent_signaling_connection_established_get_status(packet) != ERROR_CODE_SUCCESS) break;
        a2dp_subevent_signaling_connection_established_get_bd_addr(packet, phone.addr);
        phone.a2dp_cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
        phone.connected = true;
        printf("Phone connected: %s\n", bd_addr_to_str(phone.addr));
        break;

    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:
        printf("Phone SBC: %d Hz, mode %d, %d blocks, %d subbands, bitpool %d-%d\n",
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet),
               a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet));
        break;

    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
        if (a2dp_subevent_stream_established_get_status(packet) != ERROR_CODE_SUCCESS) break;
        phone.local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
        break;

    case A2DP_SUBEVENT_STREAM_STARTED:
        phone.streaming = true;
        phone.last_in_ms = 0;
        aux_need_prime = true;
        rate_t0_ms = btstack_run_loop_get_time_ms();
        rate_in0 = pwm_audio_pushed_total();
        rate_out0 = pwm_audio_played_total();
        printf("Phone started playing.\n");
        resync();
        break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:
        phone.streaming = false;
        resyncing = false;
        printf("Phone paused.\n");
        pause_booms();
        break;

    case A2DP_SUBEVENT_STREAM_RELEASED:
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        if (phone.streaming) pause_booms();
        phone.streaming = false;
        resyncing = false;
        if (sub == A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED) {
            phone.connected = false;
            printf("Phone disconnected.\n");
        }
        break;

    default:
        break;
    }
}

// --- BOOTSEL button --------------------------------------------------------
// The button shares a pin with the flash chip select, so reading it briefly
// cuts off flash. Core 0 runs this from RAM with interrupts off, and core 1
// (audio) is parked in RAM via the multicore lockout for those microseconds.
static bool __no_inline_not_in_flash_func(read_bootsel)(void) {
    const uint cs = 1;
    uint32_t flags = save_and_disable_interrupts();
    hw_write_masked(&ioqspi_hw->io[cs].ctrl, GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    for (volatile int i = 0; i < 1000; ++i) {}
    bool pressed = !(sio_hw->gpio_hi_in & (1u << cs));
    hw_write_masked(&ioqspi_hw->io[cs].ctrl, GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    restore_interrupts(flags);
    return pressed;
}

static btstack_timer_source_t button_timer, led_timer;
static int led_blinks_left; // on/off steps remaining

static void led_step(btstack_timer_source_t *t) {
    if (led_blinks_left <= 0) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); return; }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_blinks_left & 1);
    led_blinks_left--;
    btstack_run_loop_set_timer(t, 150);
    btstack_run_loop_add_timer(t);
}

static void blink(int n) {
    led_blinks_left = 2 * n;
    btstack_run_loop_remove_timer(&led_timer);
    btstack_run_loop_set_timer_handler(&led_timer, led_step);
    btstack_run_loop_set_timer(&led_timer, 1);
    btstack_run_loop_add_timer(&led_timer);
}

#define BUTTON_POLL_MS 50
static void button_poll(btstack_timer_source_t *t) {
    static bool was_down, long_done;
    static uint32_t down_ms, last_release_ms;
    static int clicks;
    btstack_run_loop_set_timer(t, BUTTON_POLL_MS);
    btstack_run_loop_add_timer(t);

    multicore_lockout_start_blocking();
    bool down = read_bootsel();
    multicore_lockout_end_blocking();
    uint32_t now = btstack_run_loop_get_time_ms();

    if (down && !was_down) { down_ms = now; long_done = false; }
    if (down && !long_done && now - down_ms > 2000) {
        long_done = true;
        clicks = 0;
        printf("Button: resync Booms\n");
        resync();
        blink(3);
    }
    if (!down && was_down && !long_done) { clicks++; last_release_ms = now; }
    if (!down && clicks && now - last_release_ms > 400) {
        if (clicks == 1) { set_aux_delay((int32_t)aux_delay_ms + 10); blink(1); }
        else { set_aux_delay((int32_t)aux_delay_ms - 10); blink(2); }
        clicks = 0;
    }
    was_down = down;
}

// Max gaps and queue depth are since the previous report.
static void print_status(void) {
    printf("status: phone %s, in %lu pkts, bad %lu, max gap %lu ms",
           phone.streaming ? "playing" : phone.connected ? "connected" : "-",
           (unsigned long)phone.packets_in, (unsigned long)phone.bad_packets, (unsigned long)phone.max_gap_ms);
    phone.max_gap_ms = 0;
    for (int i = 0; i < MAX_BOOMS; i++) {
        boom_t *b = &booms[i];
        printf(" | Boom %d %s, sent %lu, dropped %lu, max gap %lu ms, max queue %u", i + 1,
               boom_state_names[b->state], (unsigned long)b->packets_sent, (unsigned long)b->frames_dropped,
               (unsigned long)b->max_gap_ms, b->max_queue);
        b->max_gap_ms = 0;
        b->max_queue = 0;
    }
    uint32_t now = btstack_run_loop_get_time_ms(), span = now - report_start_ms;
    printf(" | AUX delay %lu ms, queued %lu ms, trim %ld ppm, jumps %lu, underruns %lu, dropped %lu, decode %lu%% CPU%s\n",
           (unsigned long)aux_delay_ms, (unsigned long)(pwm_audio_queued_frames() * 10 / 441), (long)trim_ppm,
           (unsigned long)aux_jumps,
           (unsigned long)pwm_audio_underruns(), (unsigned long)aux_dropped,
           (unsigned long)(span ? decode_us / 10 / span : 0), scanning ? " | scanning" : "");
    decode_us = 0;
    report_start_ms = now;
    if (phone.streaming && now - rate_t0_ms > 2000) {
        uint32_t dt = now - rate_t0_ms;
        printf("    rates since start: in %lu Hz, out %lu Hz\n",
               (unsigned long)((uint64_t)(pwm_audio_pushed_total() - rate_in0) * 1000 / dt),
               (unsigned long)((uint64_t)(pwm_audio_played_total() - rate_out0) * 1000 / dt));
    }
}

static void stats(btstack_timer_source_t *t) {
    print_status();
    btstack_run_loop_set_timer(t, 5000);
    btstack_run_loop_add_timer(t);
}

static void stdin_handler(char c) {
    switch (c) {
    case 's': print_status(); break;
    case 'r': resync(); break;
    case 'd':
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].a2dp_cid) a2dp_source_disconnect(booms[i].a2dp_cid);
        break;
    case 'f': forget_all(); break;
    case '+': set_aux_delay((int32_t)aux_delay_ms + 10); break;
    case '-': set_aux_delay((int32_t)aux_delay_ms - 10); break;
    default: break;
    }
}

int main(void) {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("cyw43 init failed\n");
        return 1;
    }
    cyw43_arch_gpio_put(SMPS_MODE_PIN, 1);

    // AUX output on core 1 (radio chip is already powered, so no stall later).
    const pwm_audio_carrier_t carrier = {255, false, true, false}; // 488 kHz, right antiphase
    pwm_audio_set_shaping(AUX_SHAPING);
    pwm_audio_set_gate(false);
    pwm_audio_init(PIN_AUX_LEFT, 44100, AUX_GAIN_PERCENT, &carrier);
    sbc_dec = btstack_sbc_decoder_bluedroid_init_instance(&sbc_dec_state);
    sbc_dec->configure(&sbc_dec_state, SBC_MODE_STANDARD, aux_pcm, NULL);

    hci_set_master_slave_policy(0);
    gap_set_page_timeout(PAGE_TIMEOUT_SLOTS);
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);
    l2cap_init();

    // Source side: one SBC endpoint per Boom.
    a2dp_source_init();
    a2dp_source_register_packet_handler(source_handler);
    for (int i = 0; i < MAX_BOOMS; i++) {
        avdtp_stream_endpoint_t *ep = a2dp_source_create_stream_endpoint(
            AVDTP_AUDIO, AVDTP_CODEC_SBC, source_caps[i], sizeof source_caps[i],
            source_config[i], sizeof source_config[i]);
        if (!ep) { printf("no memory for source endpoint\n"); return 1; }
        avdtp_set_preferred_sampling_frequency(ep, 44100);
        avdtp_source_register_delay_reporting_category(avdtp_local_seid(ep));
    }

    // Sink side: the iPhone sends to this endpoint.
    a2dp_sink_init();
    a2dp_sink_register_packet_handler(sink_handler);
    a2dp_sink_register_media_handler(phone_media_handler);
    sink_caps[3] = PHONE_MAX_BITPOOL;
    avdtp_stream_endpoint_t *sink_ep = a2dp_sink_create_stream_endpoint(
        AVDTP_AUDIO, AVDTP_CODEC_SBC, sink_caps, sizeof sink_caps, sink_config, sizeof sink_config);
    if (!sink_ep) { printf("no memory for sink endpoint\n"); return 1; }

    sdp_init();
    a2dp_source_create_sdp_record(sdp_source_buffer, sdp_create_service_record_handle(),
                                  AVDTP_SOURCE_FEATURE_MASK_PLAYER, NULL, NULL);
    sdp_register_service(sdp_source_buffer);
    a2dp_sink_create_sdp_record(sdp_sink_buffer, sdp_create_service_record_handle(),
                                AVDTP_SINK_FEATURE_MASK_SPEAKER, "Pico Car Audio", NULL);
    sdp_register_service(sdp_sink_buffer);

    gap_set_local_name("Pico Car Audio");
    gap_set_class_of_device(0x240414); // audio + rendering, audio/video: loudspeaker
    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

    hci_callback.callback = hci_handler;
    hci_add_event_handler(&hci_callback);
    btstack_stdin_setup(stdin_handler);

    btstack_run_loop_set_timer_handler(&button_timer, button_poll);
    btstack_run_loop_set_timer(&button_timer, BUTTON_POLL_MS);
    btstack_run_loop_add_timer(&button_timer);

    btstack_run_loop_set_timer_handler(&stats_timer, stats);
    btstack_run_loop_set_timer(&stats_timer, 5000);
    btstack_run_loop_add_timer(&stats_timer);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
}
