// Step 2b: Pico W -> two UE Booms over Bluetooth (A2DP source to each),
// metronome test signal. No PartyUp: the Pico drives both.
//
// Booms are remembered in flash (up to 2). While fewer than 2 are known, the
// Pico scans for speakers named *BOOM* (put a new Boom in pairing mode). Known
// Booms are reconnected automatically, one connection attempt at a time.
//
// The tone is SBC-encoded once and the same frames are queued for every Boom
// that is streaming; each Boom's link sends its own packets when it can.
//
// Serial commands: s = status, d = disconnect all, f = forget all Booms,
// r = resync (pause all Booms and restart them together).
// Adapted from BTstack's example/a2dp_source_demo.c.
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "btstack_tlv.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#define SMPS_MODE_PIN 1 // CYW43 WL_GPIO1: regulator PWM mode (less ripple)
#define NUM_CHANNELS 2
#define AUDIO_TIMEOUT_MS 10
#define SBC_STORAGE_SIZE 1030
#define INQUIRY_DURATION 8 // x 1.28 s
#define RETRY_MS 3000
#define MAX_BOOMS 2
#define MAX_BACKLOG_MS 100 // how far a slow Boom may hold the others back
#define RETRY_WHILE_PLAYING_MS 15000 // reconnect attempts disturb the playing Boom
#define PAGE_TIMEOUT_SLOTS 3200      // 2 s (x 0.625 ms); default is 5.12 s
#define TONE_AMPLITUDE 0.3f // about -10 dBFS
#define TLV_TAG_OLD ((uint32_t)'B' << 24 | 'O' << 16 | 'O' << 8 | 'M') // step 2's single Boom
#define TLV_TAG_BOOM(i) ((uint32_t)'B' << 24 | 'O' << 16 | 'M' << 8 | ('1' + (i)))

static uint8_t sbc_capabilities[] = {
    (AVDTP_SBC_44100 << 4) | (AVDTP_SBC_48000 << 4) | AVDTP_SBC_STEREO,
    0xFF, // all block lengths, subbands, allocation methods
    2, 53 // bitpool range
};
static uint8_t sbc_configuration[MAX_BOOMS][4];

typedef enum { B_EMPTY, B_IDLE, B_CONNECTING, B_CONNECTED, B_STREAMING } boom_state_t;
static const char *boom_state_names[] = {"empty", "idle", "connecting", "connected", "streaming"};

typedef struct {
    boom_state_t state;
    bd_addr_t addr;
    uint16_t a2dp_cid;
    uint8_t local_seid;
    int max_payload;
    uint32_t rtp_timestamp;
    uint8_t storage[SBC_STORAGE_SIZE];
    uint16_t storage_count;
    bool ready_to_send;
    uint32_t packets_sent, frames_dropped;
} boom_t;
static boom_t booms[MAX_BOOMS];

static const btstack_sbc_encoder_t *sbc;
static btstack_sbc_encoder_bluedroid_t sbc_state;
static bool sbc_ready;
static int sample_rate = 44100;

static btstack_timer_source_t audio_timer, manager_timer, stats_timer;
static bool audio_running;
// Sync gate: after a Boom starts (or 'r'), all Booms are paused and restarted
// together, and no audio is fed until every connected Boom is streaming
// again, so they all start from the same sample with empty buffers.
static bool resyncing;
static uint32_t resync_started_ms;
static uint32_t last_attempt_ms;
static uint32_t time_sent_ms, missed_acc, samples_ready;
static bool scanning;
static btstack_packet_callback_registration_t hci_callback;
static uint8_t sdp_a2dp_source_buffer[150];

// --- flash -------------------------------------------------------------------
static void tlv(const btstack_tlv_t **impl, void **ctx) { btstack_tlv_get_instance(impl, ctx); }

static void load_booms(void) {
    const btstack_tlv_t *t;
    void *c;
    tlv(&t, &c);
    for (int i = 0; i < MAX_BOOMS; i++)
        if (t->get_tag(c, TLV_TAG_BOOM(i), booms[i].addr, sizeof(bd_addr_t)) == sizeof(bd_addr_t))
            booms[i].state = B_IDLE;
    // Migrate step 2's single saved Boom into slot 1.
    bd_addr_t old;
    if (booms[0].state == B_EMPTY && t->get_tag(c, TLV_TAG_OLD, old, sizeof old) == sizeof old) {
        bd_addr_copy(booms[0].addr, old);
        booms[0].state = B_IDLE;
        t->store_tag(c, TLV_TAG_BOOM(0), old, sizeof old);
        t->delete_tag(c, TLV_TAG_OLD);
    }
    for (int i = 0; i < MAX_BOOMS; i++)
        if (booms[i].state != B_EMPTY)
            printf("Boom %d remembered: %s\n", i + 1, bd_addr_to_str(booms[i].addr));
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
    printf("Saved Boom %d (%s); it will reconnect automatically.\n", i + 1, bd_addr_to_str(booms[i].addr));
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
    printf("Forgot all Booms.\n");
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

// --- test tone ---------------------------------------------------------------
static int16_t sine_table[256];
static uint32_t gen_phase, gen_frames;

// Metronome: a 40 ms beep every 0.5 s; every 8th beep (once per 4 s) is a
// higher accent. Out-of-sync Booms sound like an echo, and an offset of whole
// beats shows up as accents that don't line up.
static void produce_audio(int16_t *pcm, int frames) {
    uint32_t beat = (uint32_t)sample_rate / 2, beep = (uint32_t)sample_rate / 25;
    for (int i = 0; i < frames; i++) {
        uint32_t pos = gen_frames % beat, n = gen_frames / beat;
        int16_t s = 0;
        if (pos < beep) {
            uint32_t freq = (n % 8 == 0) ? 1760 : 880;
            gen_phase += (uint32_t)(((uint64_t)freq << 32) / (uint32_t)sample_rate);
            s = sine_table[gen_phase >> 24];
        }
        pcm[2 * i] = pcm[2 * i + 1] = s;
        gen_frames++;
    }
}

// --- streaming ---------------------------------------------------------------
static void send_packet(boom_t *b) {
    int frame_bytes = sbc->sbc_buffer_length(&sbc_state);
    uint8_t num_frames = b->storage_count / frame_bytes;
    b->storage[0] = num_frames; // SBC media header
    a2dp_source_stream_send_media_payload_rtp(b->a2dp_cid, b->local_seid, 0, b->rtp_timestamp,
                                              b->storage, b->storage_count + 1);
    b->rtp_timestamp += num_frames * sbc->num_audio_frames(&sbc_state);
    b->storage_count = 0;
    b->ready_to_send = false;
    b->packets_sent++;
}

static void audio_timeout(btstack_timer_source_t *t) {
    btstack_run_loop_set_timer(t, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(t);
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t period = time_sent_ms ? now - time_sent_ms : AUDIO_TIMEOUT_MS;
    uint32_t n = period * sample_rate / 1000;
    missed_acc += period * sample_rate % 1000;
    while (missed_acc >= 1000) { n++; missed_acc -= 1000; }
    time_sent_ms = now;
    samples_ready += n;
    if (resyncing) { samples_ready = 0; return; }

    unsigned frames_per_sbc = sbc->num_audio_frames(&sbc_state);
    uint16_t sbc_len = sbc->sbc_buffer_length(&sbc_state);
    while (samples_ready >= frames_per_sbc) {
        // Back-pressure: if any streaming Boom has no room yet (its packet is
        // waiting for its turn on the radio), wait for it — unless we're more
        // than MAX_BACKLOG behind, then send anyway and that Boom drops a frame.
        bool all_room = true;
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].state == B_STREAMING && booms[i].storage_count + sbc_len > booms[i].max_payload)
                all_room = false;
        if (!all_room && samples_ready < (uint32_t)sample_rate * MAX_BACKLOG_MS / 1000) break;

        int16_t pcm[256 * NUM_CHANNELS];
        uint8_t frame[512];
        produce_audio(pcm, frames_per_sbc);
        sbc->encode_signed_16(&sbc_state, pcm, frame);
        samples_ready -= frames_per_sbc;
        // Same frame to every streaming Boom. A Boom whose link is backed up
        // drops the frame rather than holding the others back.
        for (int i = 0; i < MAX_BOOMS; i++) {
            boom_t *b = &booms[i];
            if (b->state != B_STREAMING) continue;
            if (b->storage_count + sbc_len <= b->max_payload) {
                memcpy(&b->storage[1 + b->storage_count], frame, sbc_len);
                b->storage_count += sbc_len;
            } else {
                b->frames_dropped++;
            }
        }
    }
    for (int i = 0; i < MAX_BOOMS; i++) {
        boom_t *b = &booms[i];
        if (b->state != B_STREAMING || b->ready_to_send) continue;
        if (b->storage_count + sbc_len > b->max_payload) {
            b->ready_to_send = true;
            a2dp_source_stream_endpoint_request_can_send_now(b->a2dp_cid, b->local_seid);
        }
    }
}

static void update_audio_timer(void) {
    bool any = false;
    for (int i = 0; i < MAX_BOOMS; i++) any |= booms[i].state == B_STREAMING;
    if (any && !audio_running) {
        time_sent_ms = missed_acc = samples_ready = 0;
        btstack_run_loop_set_timer_handler(&audio_timer, audio_timeout);
        btstack_run_loop_set_timer(&audio_timer, AUDIO_TIMEOUT_MS);
        btstack_run_loop_add_timer(&audio_timer);
        audio_running = true;
    } else if (!any && audio_running) {
        btstack_run_loop_remove_timer(&audio_timer);
        audio_running = false;
    }
}

// --- resync ------------------------------------------------------------------
static void resync(void) {
    int n = 0;
    for (int i = 0; i < MAX_BOOMS; i++) {
        boom_t *b = &booms[i];
        if (b->state != B_STREAMING) continue;
        b->state = B_CONNECTED;
        b->storage_count = 0;
        b->ready_to_send = false;
        a2dp_source_pause_stream(b->a2dp_cid, b->local_seid);
        n++;
    }
    resyncing = true;
    resync_started_ms = btstack_run_loop_get_time_ms();
    printf("Resync: pausing %d Boom(s) to restart them together.\n", n);
}

// Called when a Boom pauses or starts: once every connected Boom is paused,
// start them all; once they're all streaming, open the gate.
static void resync_step(void) {
    if (!resyncing) return;
    bool any_streaming = false, all_streaming = true, any_connected = false;
    for (int i = 0; i < MAX_BOOMS; i++) {
        if (booms[i].state == B_STREAMING) any_streaming = true;
        if (booms[i].state == B_CONNECTED) { all_streaming = false; any_connected = true; }
    }
    if (any_connected && !any_streaming) {
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].state == B_CONNECTED) a2dp_source_start_stream(booms[i].a2dp_cid, booms[i].local_seid);
        return;
    }
    if (all_streaming && any_streaming) {
        resyncing = false;
        gen_frames = 0; // restart the metronome on an accent
        samples_ready = 0;
        time_sent_ms = 0;
        printf("Resync: all Booms restarted together.\n");
    }
}

// --- connection manager ------------------------------------------------------
// Every few seconds: if nothing is mid-connect, connect the next idle Boom;
// otherwise, if fewer than 2 Booms are known, scan for a new one.
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
    uint32_t now = btstack_run_loop_get_time_ms();
    if (resyncing && now - resync_started_ms > 3000) {
        printf("Resync: timed out, starting whoever is ready.\n");
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].state == B_CONNECTED) a2dp_source_start_stream(booms[i].a2dp_cid, booms[i].local_seid);
        resyncing = false;
    }
    if (scanning) return;
    bool playing = false;
    for (int i = 0; i < MAX_BOOMS; i++) {
        if (booms[i].state == B_CONNECTING) return;
        playing |= booms[i].state == B_STREAMING;
    }
    // Paging/scanning steal radio time from a playing Boom, so try less often.
    if (now - last_attempt_ms < (playing ? RETRY_WHILE_PLAYING_MS : RETRY_MS)) return;
    for (int i = 0; i < MAX_BOOMS; i++)
        if (booms[i].state == B_IDLE) { last_attempt_ms = now; connect_boom(i); return; }
    if (known_booms() < MAX_BOOMS) { last_attempt_ms = now; start_scan(); }
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
        printf("Bluetooth up, Pico address %s\n", bd_addr_to_str(addr));
        load_booms();
        btstack_run_loop_set_timer_handler(&manager_timer, manager);
        btstack_run_loop_set_timer(&manager_timer, 100);
        btstack_run_loop_add_timer(&manager_timer);
        break;

    case HCI_EVENT_PIN_CODE_REQUEST:
        hci_event_pin_code_request_get_bd_addr(packet, addr);
        printf("Legacy PIN request from %s, answering 0000\n", bd_addr_to_str(addr));
        gap_pin_code_response(addr, "0000");
        break;

    case HCI_EVENT_USER_CONFIRMATION_REQUEST:
        hci_event_user_confirmation_request_get_bd_addr(packet, addr);
        printf("Pairing with %s (just works)\n", bd_addr_to_str(addr));
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

static void a2dp_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
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
            printf("Boom %d: connection failed (0x%02x) - off, or connected elsewhere? Retrying.\n", i + 1, status);
            booms[i].state = B_IDLE;
            booms[i].a2dp_cid = 0;
            break;
        }
        booms[i].a2dp_cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
        booms[i].state = B_CONNECTED;
        printf("Boom %d: connected.\n", i + 1);
        break;

    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
        int mode_in = a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
        int alloc = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);
        int blocks = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
        int subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
        int bitpool = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
        int rate = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
        printf("SBC: %d Hz, mode %d, %d blocks, %d subbands, bitpool %d\n", rate, mode_in, blocks, subbands, bitpool);
        // One encoder feeds every Boom, so configure it once. Two identical
        // Boom 2s negotiate the same settings.
        if (sbc_ready) break;
        btstack_sbc_channel_mode_t mode = SBC_CHANNEL_MODE_JOINT_STEREO;
        switch ((avdtp_channel_mode_t)mode_in) {
        case AVDTP_CHANNEL_MODE_STEREO: mode = SBC_CHANNEL_MODE_STEREO; break;
        case AVDTP_CHANNEL_MODE_DUAL_CHANNEL: mode = SBC_CHANNEL_MODE_DUAL_CHANNEL; break;
        case AVDTP_CHANNEL_MODE_MONO: mode = SBC_CHANNEL_MODE_MONO; break;
        default: break;
        }
        sample_rate = rate;
        sbc = btstack_sbc_encoder_bluedroid_init_instance(&sbc_state);
        sbc->configure(&sbc_state, SBC_MODE_STANDARD, blocks, subbands,
                       (btstack_sbc_allocation_method_t)(alloc - 1), rate, bitpool, mode);
        sbc_ready = true;
        break;
    }

    case A2DP_SUBEVENT_SIGNALING_DELAY_REPORT:
        printf("A Boom reports its delay as %d.%d ms\n",
               a2dp_subevent_signaling_delay_report_get_delay_100us(packet) / 10,
               a2dp_subevent_signaling_delay_report_get_delay_100us(packet) % 10);
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
        save_boom(i);
        a2dp_source_start_stream(booms[i].a2dp_cid, booms[i].local_seid);
        break;

    case A2DP_SUBEVENT_STREAM_STARTED:
        i = boom_for_cid(a2dp_subevent_stream_started_get_a2dp_cid(packet));
        if (i < 0) break;
        booms[i].max_payload = btstack_min(a2dp_max_media_payload_size(booms[i].a2dp_cid, booms[i].local_seid),
                                           SBC_STORAGE_SIZE);
        booms[i].storage_count = 0;
        booms[i].ready_to_send = false;
        booms[i].state = B_STREAMING;
        printf("Boom %d: streaming.\n", i + 1);
        update_audio_timer();
        if (resyncing) { resync_step(); break; }
        {
            // A Boom joined while another was already playing: restart all.
            int playing = 0;
            for (int k = 0; k < MAX_BOOMS; k++) playing += booms[k].state == B_STREAMING;
            if (playing > 1) resync();
        }
        break;

    case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
        i = boom_for_cid(a2dp_subevent_streaming_can_send_media_packet_now_get_a2dp_cid(packet));
        if (i >= 0 && booms[i].state == B_STREAMING) send_packet(&booms[i]);
        break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:
        i = boom_for_cid(a2dp_subevent_stream_suspended_get_a2dp_cid(packet));
        if (i < 0) break;
        booms[i].state = B_CONNECTED;
        if (resyncing) { resync_step(); break; }
        printf("Boom %d paused the stream; restarting it.\n", i + 1);
        a2dp_source_start_stream(booms[i].a2dp_cid, booms[i].local_seid);
        update_audio_timer();
        break;

    case A2DP_SUBEVENT_STREAM_RELEASED:
        i = boom_for_cid(a2dp_subevent_stream_released_get_a2dp_cid(packet));
        if (i < 0) break;
        if (booms[i].state == B_STREAMING) booms[i].state = B_CONNECTED;
        update_audio_timer();
        break;

    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        i = boom_for_cid(a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet));
        if (i < 0) break;
        printf("Boom %d: disconnected, will reconnect.\n", i + 1);
        booms[i].state = B_IDLE;
        booms[i].a2dp_cid = 0;
        update_audio_timer();
        resync_step();
        break;

    default:
        break;
    }
}

static void print_status(void) {
    printf("status:");
    for (int i = 0; i < MAX_BOOMS; i++)
        printf(" | Boom %d %s, sent %lu, dropped %lu", i + 1, boom_state_names[booms[i].state],
               (unsigned long)booms[i].packets_sent, (unsigned long)booms[i].frames_dropped);
    printf("%s\n", scanning ? " | scanning" : "");
}

static void stats(btstack_timer_source_t *t) {
    print_status();
    btstack_run_loop_set_timer(t, 5000);
    btstack_run_loop_add_timer(t);
}

static void stdin_handler(char c) {
    switch (c) {
    case 's': print_status(); break;
    case 'd':
        for (int i = 0; i < MAX_BOOMS; i++)
            if (booms[i].a2dp_cid) a2dp_source_disconnect(booms[i].a2dp_cid);
        break;
    case 'f': forget_all(); update_audio_timer(); break;
    case 'r': resync(); break;
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

    for (int i = 0; i < 256; i++)
        sine_table[i] = (int16_t)(32767.0f * TONE_AMPLITUDE * sinf(2.0f * (float)M_PI * i / 256.0f));

    hci_set_master_slave_policy(0);
    gap_set_page_timeout(PAGE_TIMEOUT_SLOTS);
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);
    l2cap_init();

    a2dp_source_init();
    a2dp_source_register_packet_handler(a2dp_handler);
    // One SBC source endpoint per Boom: BTstack gives each new connection a
    // free endpoint.
    for (int i = 0; i < MAX_BOOMS; i++) {
        avdtp_stream_endpoint_t *ep = a2dp_source_create_stream_endpoint(
            AVDTP_AUDIO, AVDTP_CODEC_SBC, sbc_capabilities, sizeof sbc_capabilities,
            sbc_configuration[i], sizeof sbc_configuration[i]);
        if (!ep) {
            printf("no memory for stream endpoint %d\n", i);
            return 1;
        }
        avdtp_set_preferred_sampling_frequency(ep, 44100);
        avdtp_source_register_delay_reporting_category(avdtp_local_seid(ep));
    }

    sdp_init();
    a2dp_source_create_sdp_record(sdp_a2dp_source_buffer, sdp_create_service_record_handle(),
                                  AVDTP_SOURCE_FEATURE_MASK_PLAYER, NULL, NULL);
    sdp_register_service(sdp_a2dp_source_buffer);

    gap_set_local_name("Pico Car Audio 00:00:00:00:00:00");
    gap_set_class_of_device(0x200408);
    gap_discoverable_control(0);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

    hci_callback.callback = hci_handler;
    hci_add_event_handler(&hci_callback);
    btstack_stdin_setup(stdin_handler);

    btstack_run_loop_set_timer_handler(&stats_timer, stats);
    btstack_run_loop_set_timer(&stats_timer, 5000);
    btstack_run_loop_add_timer(&stats_timer);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
}
