// PWM audio output for a car stereo's AUX input, with no external parts.
//
// Two GPIOs on one PWM slice (left = even pin, right = pin + 1) switch at a
// fixed carrier rate. Core 1 owns the output: it resamples the queued PCM to
// the carrier rate, applies noise shaping so 8-bit PWM sounds closer to
// 14-bit, and DMA streams the duty values into the PWM slice.
//
// Core 0 pushes 16-bit stereo frames into a single-producer/single-consumer
// ring. The amount of audio queued in the ring is the AUX delay, which is
// how the car output is lined up with the Booms later.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pico/types.h"

typedef struct {
    uint32_t out_rate_hz;   // PWM carrier = output sample rate
    uint32_t underruns;     // output samples where the ring was empty
    uint32_t queued_frames; // frames currently waiting in the ring
    uint32_t played_frames; // total frames consumed by core 1
    uint32_t fills;         // DMA half-buffers refilled
    bool gate_open;         // silence gate currently letting audio through
} pwm_audio_stats_t;

typedef struct {
    uint16_t wrap;      // counts per period - 1 (255 -> 488 kHz at 125 MHz)
    bool phase_correct; // centred pulses, half the carrier rate
    bool invert_right;  // right pin switches in antiphase to the left
    bool fast_edges;    // fast slew + 12 mA instead of slow + 2 mA
} pwm_audio_carrier_t;

// pin_left must be even; pin_right is pin_left + 1 (same PWM slice).
// gain_pct: peak-to-peak swing at full-scale input, as % of 3.3 V.
// Starts core 1. Call once from core 0.
void pwm_audio_init(uint pin_left, uint32_t input_rate_hz, uint gain_pct,
                    const pwm_audio_carrier_t *carrier);

uint32_t pwm_audio_free_frames(void);
uint32_t pwm_audio_queued_frames(void);
bool pwm_audio_push(int16_t left, int16_t right);
void pwm_audio_get_stats(pwm_audio_stats_t *out);

// Tuning (safe to call from core 0 at any time; applied at the next buffer).
void pwm_audio_set_carrier(const pwm_audio_carrier_t *carrier);
void pwm_audio_set_shaping(uint order); // 0 = plain rounding, 1, 2 (default)
uint pwm_audio_get_shaping(void);
void pwm_audio_set_gain(uint gain_pct);
// Silence gate: fade the output to a steady 0 V while nothing is playing.
// Needs >= ~150 ms of audio queued ahead to reopen before the sound arrives.
void pwm_audio_set_gate(bool enabled);
bool pwm_audio_get_gate(void);
// Gate/boot fade: duration, curve, and shaping order used while fading
// (-1 = same as normal).
void pwm_audio_set_fade(uint ms, bool cosine, int order_during_fade);

// Keep core 1 spinning instead of sleeping between buffer refills.
void pwm_audio_set_busy(bool busy);
bool pwm_audio_get_busy(void);

// Play slightly faster (+) or slower (-) to track the sender's clock and
// keep the queue (= AUX delay) steady. A few hundred ppm is inaudible.
void pwm_audio_set_rate_trim_ppm(int32_t ppm);
uint32_t pwm_audio_underruns(void);
uint32_t pwm_audio_pushed_total(void); // frames ever queued
uint32_t pwm_audio_played_total(void); // frames ever played

// Diagnostic: hold both pins at 0 (GND) or 1 (3.3 V); -1 = back to PWM.
void pwm_audio_force_level(int level);
