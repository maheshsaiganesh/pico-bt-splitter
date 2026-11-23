// Step 1: PWM audio into a car stereo's AUX input — test pattern and
// noise-hunting cycle.
//
// Wiring (Pico W pin numbers in brackets):
//   GP14 [19] -> AUX plug tip    (left)
//   GP15 [20] -> AUX plug ring   (right)
//   GND  [18] -> AUX plug sleeve (ground)
//
// Two modes (boots into the pattern; 't' over USB serial toggles):
//
// Pattern: repeating 6 s — 440 Hz left, 660 Hz right, 523 Hz both, then N
// clicks and silence. N = fade experiment number (see fade_exps[]).
//
// Noise cycle: for each carrier setup N, N short beeps then 8 s of silence
// to listen for static. Needs no computer, so it can run on a power bank.
#include <math.h>
#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pwm_audio.h"

#define PIN_LEFT 14 // right channel is GP15
#define SAMPLE_RATE 44100u
// ~3 V peak-to-peak at full scale, about a phone at full volume. The hiss is
// fixed in volts, so a big signal + low speaker volume keeps it down.
#define GAIN_PERCENT 90
#define AMPLITUDE 0.7f
#define FADE_FRAMES 441u // 10 ms fades so tone changes don't pop
#define SMPS_MODE_PIN 1  // CYW43 WL_GPIO1 -> regulator power-save pin
#define DEFAULT_SHAPING 1

typedef struct {
    pwm_audio_carrier_t carrier;
    uint shaping; // noise-shaping order used for this setup
    const char *name;
} setup_t;

// Round 1 winner: right channel in antiphase (pins never switch together).
// Round 2 varies the rest around it.
static const setup_t setups[] = {
    {{255, false, true, false}, 0, "488 kHz antiphase (round 1 winner)"},
    {{255, false, true, true}, 0, "488 kHz antiphase, sharp edges"},
    {{511, false, true, false}, 0, "244 kHz antiphase"},
    {{255, true, true, false}, 0, "244 kHz antiphase, centred pulses"},
    {{255, false, true, false}, 1, "488 kHz antiphase + 1st-order shaping"},
    {{255, false, true, false}, 2, "488 kHz antiphase + 2nd-order shaping"},
};
#define NUM_SETUPS (sizeof setups / sizeof setups[0])

static int16_t sine[256];

static inline int16_t tone(uint32_t *phase, uint32_t inc) {
    *phase += inc;
    return sine[*phase >> 24];
}

static inline uint32_t inc_for(float hz) {
    return (uint32_t)(hz * 4294967296.0f / SAMPLE_RATE);
}

static inline int32_t fade(uint32_t pos, uint32_t len) {
    if (pos < FADE_FRAMES) return (int32_t)(pos * 256 / FADE_FRAMES);
    if (pos > len - FADE_FRAMES) return (int32_t)((len - pos) * 256 / FADE_FRAMES);
    return 256;
}

// --- test pattern ----------------------------------------------------------
// Fade experiments, one per loop. Loop k plays the three tones, then k clicks,
// then silence (where the gate fades out and back in).
typedef struct {
    bool gate;
    uint fade_ms;
    bool cosine;
    int order_during_fade;
    const char *name;
} fade_exp_t;

static const fade_exp_t fade_exps[] = {
    {true, 150, false, -1, "linear 150 ms (current)"},
    {true, 150, true, -1, "smooth 150 ms"},
    {true, 400, true, -1, "smooth 400 ms"},
    {true, 150, false, 0, "linear 150 ms, no shaping while fading"},
    {true, 1, false, -1, "instant (expect a pop)"},
    {false, 150, false, -1, "gate off (hiss, no fades)"},
};
#define NUM_FADE_EXPS (sizeof fade_exps / sizeof fade_exps[0])
static bool fade_exp_running = false; // 'f' over serial to run them again
static uint32_t fade_exp;

#define PATTERN_SECONDS 6u // 3 s of tones, 3 s of clicks + silence
static uint32_t pat_n, pat_a, pat_b;

static void pattern_frame(int32_t *l, int32_t *r) {
    uint32_t seg = pat_n / SAMPLE_RATE, pos = pat_n % SAMPLE_RATE;
    int32_t env = fade(pos, SAMPLE_RATE);
    *l = *r = 0;
    if (seg == 3 && pos == 0 && fade_exp_running) {
        const fade_exp_t *e = &fade_exps[fade_exp];
        pwm_audio_set_gate(e->gate);
        pwm_audio_set_fade(e->fade_ms, e->cosine, e->order_during_fade);
        printf("fade experiment %lu: %s\n", (unsigned long)fade_exp + 1, e->name);
    }
    switch (seg) {
    case 0: *l = tone(&pat_a, inc_for(440)) * env >> 8; break;
    case 1: *r = tone(&pat_a, inc_for(660)) * env >> 8; break;
    case 2: *l = *r = tone(&pat_a, inc_for(523.25f)) * env >> 8; break;
    case 3: {
        uint32_t clicks = fade_exp_running ? fade_exp + 1 : 1;
        uint32_t slot = SAMPLE_RATE * 15 / 100; // a click every 150 ms
        if (pos / slot < clicks && pos % slot < 220) *l = *r = tone(&pat_b, inc_for(2000));
        break;
    }
    default: break;
    }
    if (++pat_n == PATTERN_SECONDS * SAMPLE_RATE) {
        pat_n = 0;
        if (fade_exp_running && ++fade_exp == NUM_FADE_EXPS) fade_exp = 0;
    }
}

// --- noise cycle -----------------------------------------------------------
#define BEEP_FRAMES (SAMPLE_RATE / 10)  // 100 ms beep
#define BEEP_SLOT (SAMPLE_RATE / 4)     // one beep every 250 ms
#define LISTEN_FRAMES (SAMPLE_RATE * 9) // ~1 s gap + 8 s of silence
static uint32_t cyc_setup, cyc_n, cyc_ph;

static void cycle_frame(int32_t *l, int32_t *r) {
    if (cyc_n == 0) {
        pwm_audio_set_carrier(&setups[cyc_setup].carrier);
        pwm_audio_set_shaping(setups[cyc_setup].shaping);
        printf("noise cycle: setup %lu = %s\n", (unsigned long)cyc_setup + 1,
               setups[cyc_setup].name);
    }
    uint32_t beeps = cyc_setup + 1, beep_end = beeps * BEEP_SLOT;
    *l = *r = 0;
    if (cyc_n < beep_end) {
        uint32_t pos = cyc_n % BEEP_SLOT;
        if (pos < BEEP_FRAMES)
            *l = *r = tone(&cyc_ph, inc_for(1000)) * fade(pos, BEEP_FRAMES) >> 9;
    }
    if (++cyc_n == beep_end + LISTEN_FRAMES) {
        cyc_n = 0;
        if (++cyc_setup == NUM_SETUPS) cyc_setup = 0;
    }
}

int main(void) {
    stdio_init_all();
    if (cyw43_arch_init() != 0) printf("cyw43 init failed (LED disabled)\n");
    // The radio chip only powers up (~0.5 s firmware load) on first use. Do it
    // now, before audio starts, so core 0 never stalls while audio is playing.
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    // High = regulator in PWM mode: much less ripple on 3.3 V, which the PWM
    // audio swings to.
    cyw43_arch_gpio_put(SMPS_MODE_PIN, 1);

    for (int i = 0; i < 256; i++)
        sine[i] = (int16_t)(32767.0f * AMPLITUDE * sinf(2.0f * (float)M_PI * i / 256.0f));

    // Defaults from the static hunt: 488 kHz, right channel in antiphase,
    // 1st-order shaping (2nd order added audible hiss on the Boom).
    bool cycle_mode = false;
    pwm_audio_set_shaping(DEFAULT_SHAPING);
    // Silence gate off: every fade we tried made a 'bzzt' on the Boom and an
    // instant switch pops. The steady mild hiss was preferred. 'g' to try it.
    pwm_audio_set_gate(false);
    pwm_audio_init(PIN_LEFT, SAMPLE_RATE, GAIN_PERCENT, &setups[0].carrier);

    // Serial: t = pattern/noise cycle, s = regulator mode, 0/1/2 = shaping
    // order, b = busy cores, g = silence gate, f = fade experiments on/off, m = mute, + / - = gain, z / h = hold pins at GND / 3.3 V, p = PWM.
    bool smps_pwm = true, muted = false;
    int forced = -1;
    uint gain = GAIN_PERCENT;

    absolute_time_t next_report = make_timeout_time_ms(2000);
    bool led = false;
    absolute_time_t next_led = get_absolute_time();

    for (;;) {
        // Keep the queue short in cycle mode so setup changes line up with
        // the beeps you hear.
        uint32_t limit = cycle_mode ? 1024 : 16384;
        while (pwm_audio_queued_frames() < limit) {
            int32_t l, r;
            if (cycle_mode) cycle_frame(&l, &r);
            else pattern_frame(&l, &r);
            if (muted) l = r = 0;
            pwm_audio_push((int16_t)l, (int16_t)r);
        }

        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == 't') {
                cycle_mode = !cycle_mode;
                cyc_n = cyc_setup = 0;
                pwm_audio_set_gate(false);
                if (!cycle_mode) {
                    pwm_audio_set_carrier(&setups[0].carrier);
                    pwm_audio_set_shaping(DEFAULT_SHAPING);
                }
            }
            else if (c == 's') { smps_pwm = !smps_pwm; cyw43_arch_gpio_put(SMPS_MODE_PIN, smps_pwm); }
            else if (c >= '0' && c <= '2') pwm_audio_set_shaping((uint)(c - '0'));
            else if (c == 'm') muted = !muted;
            else if (c == 'g') pwm_audio_set_gate(!pwm_audio_get_gate());
            else if (c == 'f') fade_exp_running = !fade_exp_running;
            else if (c == 'b') pwm_audio_set_busy(!pwm_audio_get_busy());
            else if (c == '+' && gain < 100) pwm_audio_set_gain(gain += 10);
            else if (c == '-' && gain > 10) pwm_audio_set_gain(gain -= 10);
            else if (c == 'z') pwm_audio_force_level(forced = 0);
            else if (c == 'h') pwm_audio_force_level(forced = 1);
            else if (c == 'p') pwm_audio_force_level(forced = -1);
            printf("settings: busy %s, %s, gate %s, regulator %s, shaping %u, gain %u%%, %s\n",
                   pwm_audio_get_busy() ? "on" : "off",
                   cycle_mode ? "noise cycle" : "pattern", pwm_audio_get_gate() ? "on" : "off",
                   smps_pwm ? "PWM (quiet)" : "power-save", pwm_audio_get_shaping(),
                   gain, muted ? "muted" : "unmuted");
            if (forced >= 0) printf("  (outputs held %s)\n", forced ? "at 3.3 V" : "at GND");
        }

        if (time_reached(next_led)) {
            led = !led;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
            next_led = make_timeout_time_ms(500);
        }
        if (time_reached(next_report)) {
            pwm_audio_stats_t s;
            pwm_audio_get_stats(&s);
            printf("pwm: out %lu Hz, queued %lu, underruns %lu, gate %s\n",
                   (unsigned long)s.out_rate_hz, (unsigned long)s.queued_frames,
                   (unsigned long)s.underruns, s.gate_open ? "open" : "closed");
            next_report = make_timeout_time_ms(2000);
        }
        // Busy mode: spin instead of sleeping so current draw stays flat.
        if (pwm_audio_get_busy()) busy_wait_us_32(1000);
        else sleep_ms(1);
    }
}
