#include "pwm_audio.h"

#include <math.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/resets.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

// --- ring buffer (core 0 writes, core 1 reads) -----------------------------
// 32768 frames = ~743 ms at 44.1 kHz: room for a long AUX delay plus phone bursts.
#define RING_FRAMES 32768u
#define RING_MASK (RING_FRAMES - 1u)
static uint32_t ring[RING_FRAMES]; // (uint16 right << 16) | uint16 left
static volatile uint32_t ring_head; // written by core 0
static volatile uint32_t ring_tail; // written by core 1

uint32_t pwm_audio_queued_frames(void) { return ring_head - ring_tail; }
uint32_t pwm_audio_free_frames(void) { return RING_FRAMES - pwm_audio_queued_frames(); }

// --- silence gate ------------------------------------------------------------
// With no audio, the output fades down to a steady 0 V (completely silent)
// instead of idling at a hissy 50% PWM. Core 0 marks the newest non-silent
// frame as it's queued, so the gate reopens before that frame is played —
// the queue (the AUX delay) is the look-ahead.
#define GATE_THRESHOLD 4                        // |sample| above this = audio
static volatile bool gate_enabled;
static volatile bool heard_audio;
static volatile uint32_t last_active;           // ring index of newest loud frame
static uint32_t gate_hold_frames;               // stay open this long after audio
static volatile bool gate_open_now;

bool pwm_audio_push(int16_t left, int16_t right) {
    uint32_t head = ring_head;
    if (head - ring_tail >= RING_FRAMES) return false;
    if (left > GATE_THRESHOLD || left < -GATE_THRESHOLD ||
        right > GATE_THRESHOLD || right < -GATE_THRESHOLD) {
        last_active = head;
        heard_audio = true;
    }
    ring[head & RING_MASK] = ((uint32_t)(uint16_t)right << 16) | (uint16_t)left;
    __dmb();
    ring_head = head + 1;
    return true;
}

// --- output state (core 1 only unless noted) ------------------------------
#define DMA_WORDS 1024u      // one PWM period per word
#define RAMP_ONE (1 << 24)   // fade position: 0 = 0 V, RAMP_ONE = midpoint

static uint32_t dma_buf[2][DMA_WORDS];
static int dma_ch[2];
static uint pwm_slice;
static uint pin_l;
static uint32_t input_rate;

static pwm_audio_carrier_t carrier;          // active (core 1)
static pwm_audio_carrier_t carrier_req;      // requested (core 0)
static volatile bool carrier_pending;
static volatile uint32_t out_rate;
static volatile uint gain_percent = 40;
static volatile bool gain_pending;
static uint32_t wrap;
static int32_t gain_mult; // int16 sample -> Q16 PWM counts

static uint32_t step_q32, phase_q32;
static uint32_t base_step_q32;          // input/output rate ratio, Q32
static volatile int32_t rate_trim_q20;  // speed trim, parts per 2^20 (~ppm)
static int32_t prev_l, prev_r, cur_l, cur_r;
static int32_t center_target_q16;
static int32_t ramp_pos;                 // 0..RAMP_ONE
static volatile uint ramp_ms = 150;      // fade time (boot and gate)
static volatile bool ramp_cosine;        // raised-cosine instead of linear
static volatile int ramp_order = -1;     // shaping order during fades (-1 = same)
static uint16_t cos_curve[257];          // (1 - cos(pi x)) / 2, Q16
static int32_t e1l, e2l, e1r, e2r;
static volatile uint32_t underruns;
static volatile uint32_t fills;
static volatile uint shaping_order = 2;
static bool started; // don't count underruns before the first frame arrives
static volatile bool busy_cores;

// Error feedback quantizer. Order 2: q = x + (1 - z^-1)^2 * e
static inline uint32_t shape(int32_t x_q16, int32_t *e1, int32_t *e2, uint order) {
    int32_t u = order == 2 ? x_q16 - 2 * *e1 + *e2 : order == 1 ? x_q16 - *e1 : x_q16;
    int32_t q = (u + 0x8000) >> 16;
    if (q < 0) q = 0;
    else if (q > (int32_t)wrap) q = (int32_t)wrap;
    int32_t e = (q << 16) - u;
    // Keep the loop stable if we ever clip.
    if (e > 0x18000) e = 0x18000;
    else if (e < -0x18000) e = -0x18000;
    *e2 = *e1;
    *e1 = e;
    return (uint32_t)q;
}

static void apply_gain(void) {
    // full-scale int16 -> +/- (gain% of range)/2 counts, in Q16
    int32_t amp_counts = (int32_t)((wrap + 1) * gain_percent / 200);
    gain_mult = amp_counts * 2; // 32768 * gain_mult = amp_counts << 16
}

// Core 1, or core 0 before core 1 starts.
static void apply_carrier(bool first) {
    wrap = carrier.wrap;
    if (!first) {
        // Leaving phase-correct mode can strand the counter counting down
        // through all 65536 counts, so start the PWM block from scratch.
        reset_block_num(RESET_PWM);
        unreset_block_num_wait_blocking(RESET_PWM);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&cfg, 1);
        pwm_init(pwm_slice, &cfg, false);
    }
    pwm_set_wrap(pwm_slice, wrap);
    pwm_set_phase_correct(pwm_slice, carrier.phase_correct);
    pwm_set_output_polarity(pwm_slice, false, carrier.invert_right);
    uint32_t period = carrier.phase_correct ? 2 * wrap : wrap + 1;
    out_rate = clock_get_hz(clk_sys) / period;
    base_step_q32 = (uint32_t)(((uint64_t)input_rate << 32) / out_rate);
    step_q32 = base_step_q32;
    center_target_q16 = (int32_t)(((wrap + 1) / 2) << 16);
    ramp_pos = first ? 0 : RAMP_ONE;
    if (!first) pwm_set_enabled(pwm_slice, true);
    e1l = e2l = e1r = e2r = 0;
    apply_gain();
}

// Fade position (0..RAMP_ONE) -> midpoint fraction, Q16 (0..65536).
static inline uint32_t curve(int32_t pos, bool cosine) {
    if (!cosine) return (uint32_t)pos >> 8;
    uint32_t idx = (uint32_t)pos >> 16, f = (uint32_t)pos & 0xffff;
    if (idx >= 256) return 65536;
    uint32_t a = cos_curve[idx], b = idx == 255 ? 65536 : cos_curve[idx + 1];
    return a + (((b - a) * f) >> 16);
}

static void __not_in_flash_func(fill)(uint32_t *buf) {
    fills++;
    if (carrier_pending) {
        carrier = carrier_req;
        carrier_pending = false;
        apply_carrier(false);
    }
    if (gain_pending) {
        gain_pending = false;
        apply_gain();
    }
    step_q32 = base_step_q32 + (uint32_t)(((int64_t)base_step_q32 * rate_trim_q20) >> 20);
    uint order = shaping_order;
    bool invert_r = carrier.invert_right;
    uint32_t tail_now = ring_tail;
    bool open = !gate_enabled ||
                (heard_audio && (int32_t)(last_active + gate_hold_frames - tail_now) > 0);
    gate_open_now = open;
    int32_t goal = open ? RAMP_ONE : 0;
    int32_t step = (int32_t)((uint64_t)RAMP_ONE * 1000 / ((uint64_t)ramp_ms * out_rate));
    if (step < 1) step = 1;
    bool cosine = ramp_cosine;
    int r_order = ramp_order;

    // Swing scales with the midpoint so fades never clip. Use the lower of the
    // position now and at the end of this buffer.
    int32_t span = step * (int32_t)DMA_WORDS;
    int32_t end = ramp_pos < goal ? (ramp_pos + span < goal ? ramp_pos + span : goal)
                                  : (ramp_pos - span > goal ? ramp_pos - span : goal);
    int32_t low = end < ramp_pos ? end : ramp_pos;
    int32_t g = (int32_t)(((int64_t)gain_mult * curve(low, cosine)) >> 16);

    for (uint32_t i = 0; i < DMA_WORDS; i++) {
        uint32_t np = phase_q32 + step_q32;
        if (np < phase_q32) { // crossed into the next input frame
            prev_l = cur_l;
            prev_r = cur_r;
            uint32_t tail = ring_tail;
            if (ring_head != tail) {
                uint32_t f = ring[tail & RING_MASK];
                cur_l = (int16_t)(f & 0xffff);
                cur_r = (int16_t)(f >> 16);
                ring_tail = tail + 1;
                started = true;
            } else {
                cur_l = cur_r = 0;
                if (started) underruns++;
            }
        }
        phase_q32 = np;

        int32_t frac = (int32_t)(np >> 17); // 15-bit fraction
        int32_t l = prev_l + (((cur_l - prev_l) * frac) >> 15);
        int32_t r = prev_r + (((cur_r - prev_r) * frac) >> 15);

        int32_t c = center_target_q16;
        uint o = order;
        if (ramp_pos != goal) {
            if (ramp_pos < goal) { ramp_pos += step; if (ramp_pos > goal) ramp_pos = goal; }
            else { ramp_pos -= step; if (ramp_pos < goal) ramp_pos = goal; }
        }
        if (ramp_pos != RAMP_ONE) {
            c = (int32_t)(((uint32_t)(center_target_q16 >> 8) * curve(ramp_pos, cosine)) >> 8);
            if (ramp_pos != goal) {
                // While fading, keep the midpoint's fractional part away from
                // whole PWM counts, where the shaper's flicker between
                // neighbouring counts slows into the audible range. Mapping the
                // fraction into [0.04, 0.96] keeps it above ~19.5 kHz.
                c = (c & ~0xffff) + 2621 + (int32_t)(((uint32_t)c & 0xffff) * 60293u >> 16);
                if (r_order >= 0) o = (uint)r_order;
            }
        }

        uint32_t ql = shape(c + l * g, &e1l, &e2l, o);
        uint32_t qr = shape(c + r * g, &e1r, &e2r, o);
        // Inverted output is high for (wrap + 1 - cc) counts.
        if (invert_r) qr = wrap + 1 - qr;
        buf[i] = (qr << 16) | ql; // CC register: B in high half, A in low half
    }
}

static void __not_in_flash_func(dma_irq)(void) {
    for (int i = 0; i < 2; i++) {
        uint32_t mask = 1u << dma_ch[i];
        if (dma_hw->ints1 & mask) {
            dma_hw->ints1 = mask;
            fill(dma_buf[i]);
            dma_channel_set_read_addr(dma_ch[i], dma_buf[i], false);
        }
    }
}

static void core1_main(void) {
    // Let core 0 pause us while it writes flash (BTstack saves pairings);
    // otherwise core 1 would crash fetching code from flash mid-write.
    multicore_lockout_victim_init();
    for (int i = 0; i < 2; i++) {
        dma_channel_config c = dma_channel_get_default_config(dma_ch[i]);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pwm_get_dreq(pwm_slice));
        channel_config_set_chain_to(&c, dma_ch[i ^ 1]);
        dma_channel_configure(dma_ch[i], &c, &pwm_hw->slice[pwm_slice].cc,
                              dma_buf[i], DMA_WORDS, false);
        dma_channel_set_irq1_enabled(dma_ch[i], true);
    }
    fill(dma_buf[0]);
    fill(dma_buf[1]);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_irq);
    irq_set_enabled(DMA_IRQ_1, true);
    dma_channel_start(dma_ch[0]);
    // Busy mode: spin between refills instead of sleeping, so the current
    // draw (and with it the 3.3 V rail the PWM swings to) stays flat.
    for (;;) {
        if (busy_cores) tight_loop_contents();
        else __wfi();
    }
}

static void apply_edges(bool fast) {
    for (uint p = pin_l; p <= pin_l + 1; p++) {
        gpio_set_slew_rate(p, fast ? GPIO_SLEW_RATE_FAST : GPIO_SLEW_RATE_SLOW);
        gpio_set_drive_strength(p, fast ? GPIO_DRIVE_STRENGTH_12MA : GPIO_DRIVE_STRENGTH_2MA);
    }
}

void pwm_audio_init(uint pin_left, uint32_t input_rate_hz, uint gain_pct,
                    const pwm_audio_carrier_t *c) {
    pin_l = pin_left;
    pwm_slice = pwm_gpio_to_slice_num(pin_left);
    input_rate = input_rate_hz;
    gate_hold_frames = input_rate_hz * 3 / 10; // 300 ms
    gain_percent = gain_pct;
    carrier = *c;

    for (int i = 0; i <= 256; i++)
        cos_curve[i] = (uint16_t)(i == 256 ? 65535 : (1.0 - cos(M_PI * i / 256.0)) * 32768.0);
    for (uint p = pin_left; p <= pin_left + 1; p++) gpio_set_function(p, GPIO_FUNC_PWM);
    apply_edges(carrier.fast_edges);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_init(pwm_slice, &cfg, false);
    pwm_set_both_levels(pwm_slice, 0, 0);
    apply_carrier(true);
    pwm_set_enabled(pwm_slice, true);

    dma_ch[0] = dma_claim_unused_channel(true);
    dma_ch[1] = dma_claim_unused_channel(true);
    multicore_launch_core1(core1_main);
}

void pwm_audio_set_carrier(const pwm_audio_carrier_t *c) {
    apply_edges(c->fast_edges);
    carrier_req = *c;
    __dmb();
    carrier_pending = true;
}

void pwm_audio_get_stats(pwm_audio_stats_t *out) {
    out->out_rate_hz = out_rate;
    out->underruns = underruns;
    out->queued_frames = pwm_audio_queued_frames();
    out->played_frames = ring_tail;
    out->fills = fills;
    out->gate_open = gate_open_now;
}

void pwm_audio_set_shaping(uint order) { shaping_order = order > 2 ? 2 : order; }
uint pwm_audio_get_shaping(void) { return shaping_order; }

void pwm_audio_set_gain(uint gain_pct) {
    gain_percent = gain_pct;
    gain_pending = true;
}

void pwm_audio_force_level(int level) {
    for (uint p = pin_l; p <= pin_l + 1; p++) {
        if (level >= 0) {
            gpio_put(p, level);
            gpio_set_dir(p, GPIO_OUT);
            gpio_set_function(p, GPIO_FUNC_SIO);
        } else {
            gpio_set_function(p, GPIO_FUNC_PWM);
        }
    }
}

void pwm_audio_set_gate(bool enabled) { gate_enabled = enabled; }
bool pwm_audio_get_gate(void) { return gate_enabled; }

void pwm_audio_set_fade(uint ms, bool cosine, int order_during_fade) {
    ramp_ms = ms ? ms : 1;
    ramp_cosine = cosine;
    ramp_order = order_during_fade;
}

void pwm_audio_set_busy(bool busy) { busy_cores = busy; }
bool pwm_audio_get_busy(void) { return busy_cores; }

void pwm_audio_set_rate_trim_ppm(int32_t ppm) {
    // ppm -> parts per 2^20 (1 ppm ~ 1.049)
    rate_trim_q20 = (int32_t)(((int64_t)ppm * 1048576) / 1000000);
}

uint32_t pwm_audio_underruns(void) { return underruns; }

uint32_t pwm_audio_pushed_total(void) { return ring_head; }
uint32_t pwm_audio_played_total(void) { return ring_tail; }
