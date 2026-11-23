# Development notes

How the project got to where it is, what was measured, and the gotchas found
along the way. For setup and usage, see [README.md](README.md).

```
Phone ──BT A2DP──▶ Pico W ──BT A2DP──▶ Speaker 1
                      ├────BT A2DP──▶ Speaker 2
                      └──PWM on GP14/GP15, delayed──▶ AUX ──▶ car stereo
```

## Why the Pico drives both speakers itself

The speakers (UE Boom 2) have their own "PartyUp" linking, set up through a
phone app or a button combo. That didn't fit here:

- These units only remember one paired device; pairing anything new needs a
  reset.
- The button-combo PartyUp didn't work.

So each speaker is reset once and paired only with the Pico, the phone pairs
with the Pico, and the Pico sends the same audio to both speakers itself.

## Programs

| Target | What it does |
|---|---|
| `car_audio` | Step 1: PWM test pattern into the AUX input, plus the noise-hunting experiments |
| `boom_test` | Step 2: Pico → one or two speakers over Bluetooth, test tones / metronome |
| `hub` | Steps 3–4: the real thing — phone → Pico → both speakers + delayed AUX |

## Steps and findings

### 1. PWM audio into AUX (no DAC, no resistors, no capacitors)

- 488 kHz PWM (125 MHz / 256), DMA'd one word per PWM period into the slice's
  compare register (both channels in one write). Core 1 does linear-interp
  resampling to the carrier rate and error-feedback noise shaping.
- Static hunt, listening on a speaker's AUX input:
  - Regulator in PWM mode (CYW43 WL_GPIO1 high) — significantly less noise.
  - Pins held at GND: silent. Pins held at 3.3 V: mild hiss (supply noise).
    PWM at 50 %: obvious static → the switching itself was the main source.
  - Right channel in **antiphase** (pins never switch together) — quietest.
  - Carrier 488 vs 244 vs 122 kHz, soft vs sharp edges: no audible difference.
  - 2nd-order noise shaping added audible hiss; 1st-order didn't. Chosen: 1st.
  - Output level ~3 V p-p at full scale (`GAIN_PERCENT` 90): the hiss is
    fixed in volts, so a big signal + lower volume keeps it down.
- **Silence gate** (fade the output to 0 V when nothing plays): every fade
  tried (linear/smooth, 150/400 ms, with/without shaping) made an audible
  "bzzt"; an instant switch pops. Left **off**; code kept (`pwm_audio_set_gate`).
- **Buzz = USB data.** A faint buzz rising with volume was the USB link to the
  computer (gone on a power bank). Car builds (`./build.sh car`) turn USB off.
  A power bank also avoids a USB + AUX ground loop (alternator whine).
- Gotchas:
  - `cyw43_arch_init()` doesn't power the radio; the first cyw43 call does
    (~0.5 s firmware load). Do it before starting audio or core 0 stalls.
  - Turning PWM phase-correct mode off can leave the counter counting down
    through 65536 counts (audio at ~0.4 % speed). `apply_carrier()` resets the
    whole PWM block on every change.

### 2. Pico → speakers over Bluetooth (A2DP source)

- Speakers found by inquiry (name contains "BOOM"), paired "just works",
  addresses saved in flash (BTstack TLV tags `BOM1`/`BOM2`). A speaker that's
  switched on reconnects to the Pico by itself.
- **Sync:** without care the two speakers ended up ~1 s apart (the first was
  starved while the Pico paged the second). Fix: whenever a speaker joins, all
  are paused and restarted together (`resync()`). Result: ~1 ms apart by ear.
- Paging/scanning steals radio time from a playing speaker → page timeout 2 s,
  reconnect attempts every 15 s while something plays.
- The speakers send no A2DP delay report.

### 3. Phone → Pico → both speakers

- The Pico is an A2DP sink ("Pico Car Audio") to the phone and an A2DP source
  to each speaker. All links are pinned to one SBC configuration (44.1 kHz,
  joint stereo, 16 blocks, 8 subbands, loudness), so the phone's SBC frames
  are **forwarded untouched** — no decode/re-encode.
- **Radio throughput was the limit.** First version: speakers got only ~80 %
  of frames at bitpool 53 (crackles, dropouts). Fixes:
  1. Fill outgoing packets to the speakers' max payload (883 bytes) instead of
     one-out-per-one-in.
  2. `MAX_NR_CONTROLLER_ACL_BUFFERS` 3 → 6 so packets for both speakers
     overlap at the controller.

  Result at full quality (bitpool 53): 0 drops, queues peak at 3–4 of 8.
  Bitpool 35 also fixed the drops but sounded noticeably worse;
  `PHONE_MAX_BITPOOL` remains as a fallback knob.
- Phone packets arrive up to ~80 ms apart (nominal 23 ms); the speakers cope.
- Gotcha: the RTP CSRC count is the **low** nibble of byte 0.
- "Lossless" streaming on the phone makes no difference: Bluetooth re-encodes.

### 4. Delayed AUX output

- The phone's SBC frames are also decoded (bluedroid, ~21 % of core 0) into
  the `pwm_audio` ring on core 1. The ring (~371 ms) is the delay line.
- AUX delay = frames queued right after each phone packet. Held on target by a
  PI speed trim (±1000 ppm, I-term ±300 ppm, frozen for the first 10 s) plus
  jumps (insert silence / skip audio) when more than 20 ms off.
- Gotchas:
  - Prime the delay at the phone's **first media packet**, not at
    STREAM_STARTED — the first audio arrives tens of ms later.
  - The phone sends slightly slow for the first ~20 s after play; that's not a
    clock difference, so the I-term must not learn from it.
- Calibrated by ear on snare hits against a wired speaker: **180–190 ms**.
  Saved in flash (TLV tag `AUXD`); adjustable with the BOOTSEL button.
- Reading BOOTSEL briefly disables flash (it shares the flash chip-select
  pin), so core 1 is parked in RAM via the multicore lockout during the read.
  A flash save stalls core 0 briefly (one speaker dropped 7 frames once).

### 5. In the car

Car build (USB off) on a power bank, AUX into the car stereo: works.

### 6. On the road (first drives)

Speakers drifted apart within minutes and the AUX timing against them was
inconsistent. Findings from a drive with the logging build (USB on, Pico
powered by a laptop):

- **Radio load is higher in a moving car.** At bitpool 53 both speakers
  sometimes lost ~25 % of frames. **Bitpool 45** (`PHONE_MAX_BITPOOL`) fixed
  that with little audible loss; the per-speaker queue went 8 → 16 packets.
- **AUX ring too small for a car-tuned delay.** The delay had been tuned up
  to ~290 ms in the car; with phone bursts on top the 371 ms ring overflowed
  (crackles). Ring doubled to 32768 frames (~743 ms); max delay now 500 ms.
- **AUX steering chased phone burstiness.** Phone packets arrive up to
  ~170 ms apart in the car, so the 1 s average of the queue depth swung
  ±50 ms and the trim slammed ±1000 ppm, with frequent jumps — the AUX
  wandered around the speakers. Now: ~8 s smoothing, P 60 ppm per 10 ms,
  total ±500 ppm, jumps only when > 80 ms off and never in the first 30 s
  (the phone sends slow then; the speakers drain too, so it isn't a real
  offset). Button steps now apply exactly 10 ms at once (silence in / skip).
- **Speaker-to-speaker drift.** No shared clock, and a frame dropped for only
  one speaker moves it ~3 ms ahead for good. The speakers are now restarted
  together automatically in quiet gaps (≥ 400 ms below ~−60 dBFS, typically
  between songs), at most every 3 minutes.
- Tried and switched off by default (serial toggles for testing):
  dropping a frame for both speakers whenever one is full (`y`) — sounded
  worse, every congestion glitch hit both; resync as soon as a speaker stalls
  ≥ 200 ms (`t`). Auto resync can be toggled with `a`.

After these, the Pico itself dropped nothing. Some crackling remained;
not yet narrowed down to the speakers (phone gaps passed straight through —
a steady-pacing buffer would help) or the AUX (USB-on build).

## Open questions

- Remaining occasional crackles (see above): speakers, AUX, or both?
- Whether the automatic resync on silence keeps up during long stretches with
  no gaps (DJ mixes, crossfade on). Hold BOOTSEL for 2 s to resync by hand.
- Podcasts: a resync in a pause in speech could clip the first syllable on
  the speakers. Raise `SILENCE_RESYNC_MS` if that's noticeable.
- Speaker volume is per speaker (their own buttons); no AVRCP volume control.
