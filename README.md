# pico-bt-splitter

Play your phone's audio through a car stereo **and** two Bluetooth speakers at
the same time, in sync, using nothing but a Raspberry Pi Pico W and an AUX
cable.

Phones send media to one Bluetooth device at a time. This firmware makes the
Pico W look like an ordinary Bluetooth speaker to the phone, then fans the
audio out:

```
Phone ──Bluetooth──▶ Pico W ──Bluetooth──▶ Speaker 1
                        ├──────Bluetooth──▶ Speaker 2
                        └──AUX cable (delayed to match)──▶ Car stereo
```

- **Works with any app** (Spotify, YouTube, podcasts…): the phone just sees a
  speaker called "Pico Car Audio".
- **Both speakers in sync** without the speakers' own linking feature — the
  Pico restarts them together (~1 ms apart by ear).
- **Full SBC quality, no re-encoding:** the phone's compressed audio is
  forwarded to the speakers untouched.
- **AUX output with no extra parts:** PWM straight from two GPIO pins into the
  car's AUX input — no DAC, resistors or capacitors.
- **Car and speakers line up:** the AUX output is delayed to match the
  speakers' Bluetooth latency, with clock-drift correction, and the delay is
  fine-tuned with the Pico's BOOTSEL button.
- **Remembers everything:** speakers and phone reconnect automatically.

Tested with two UE Boom 2 speakers and an iPhone. Other A2DP speakers should
work; speaker discovery currently looks for names containing "BOOM"
(`is_boom_name()` in `src/hub.c`).

## Hardware

- Raspberry Pi Pico W (RP2040 + CYW43439)
- A 3.5 mm AUX cable, cut open, with its three wires connected to the Pico
- A power bank (or a USB port) and a micro-USB cable for power

### Wiring

| Pico W pin | Signal | AUX plug |
|-----------:|--------|----------|
| 19 (GP14)  | left   | tip      |
| 20 (GP15)  | right  | ring     |
| 18 (GND)   | ground | sleeve   |

Typical AUX cable colours: red = right, white/blue/green = left, bare = ground
— check yours. The coloured wires are often enamel-coated; burn the coating
off with the soldering iron or the joint won't conduct.

The PWM swings 0–3.3 V, about what a phone's headphone output does at full
volume, and car AUX inputs are AC-coupled, so no filter parts are needed. A
power bank is the quietest supply (USB data traffic is audible, and a car USB
port can add a ground loop).

## Building

Needs the [Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.x, the Arm GNU
toolchain and `picotool`. `build.sh` expects the SDK in
`~/.pico-sdk/sdk-2.3.1` and the toolchain in
`~/.pico-sdk/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi`; edit the
two paths at the top for your setup.

```sh
TARGET=hub ./build.sh flash       # development build (USB serial log), flash it
TARGET=hub ./build.sh car flash   # car build: USB off, flash it
```

`flash` reboots a running Pico into BOOTSEL by itself while USB is on. A car
build has USB off: to reflash it, hold BOOTSEL while plugging the Pico in.

## Setup

1. **Speakers:** reset each speaker if it only remembers one device, then put
   it in pairing mode with the Pico running. The Pico finds it, pairs, and
   saves it (up to two speakers).
2. **Phone:** pair with **Pico Car Audio** and choose it as the audio output.
3. **Car:** plug the AUX cable in and set the stereo to AUX.

From then on: power the Pico, switch on the speakers, play.

## Controls

**BOOTSEL button** (LED confirms):

| Action | Effect | LED |
|---|---|---|
| 1 press | AUX 10 ms later | 1 blink |
| 2 quick presses | AUX 10 ms earlier | 2 blinks |
| Hold 2 s | Restart both speakers together (resync) | 3 blinks |

Tune by ear on drum hits: if the car speakers hit first, press once; if the
Bluetooth speakers hit first, double-press. The setting is saved.

**USB serial** (development build, 115200 baud): `s` status, `+`/`-` AUX delay,
`r` resync, `d` disconnect speakers, `f` forget both speakers.

## How it works

- **Core 0** runs BTstack: an A2DP sink for the phone and an A2DP source per
  speaker. Every link uses the same SBC settings (44.1 kHz, joint stereo,
  16 blocks, 8 subbands, bitpool ≤ 53), so incoming SBC frames are copied
  straight into each speaker's packet queue. The frames are also decoded for
  the AUX output.
- **Core 1** owns the AUX output: decoded audio goes into a ~371 ms ring
  buffer (the delay line), is resampled to a 488 kHz PWM carrier with
  1st-order noise shaping, and is DMA'd into the PWM slice. The right channel
  switches in antiphase to the left, which noticeably reduces noise.
- A PI controller trims the AUX playback speed by a few hundred ppm so the
  delay stays constant despite the phone's and the Pico's clocks differing.

[STATUS.md](STATUS.md) has the development notes: measurements, dead ends and
gotchas (radio throughput limits, sync, noise hunting, drift).

## Source layout

| Path | Purpose |
|---|---|
| `src/hub.c` | Main firmware: phone → speakers + AUX |
| `src/pwm_audio.c/.h` | PWM audio output engine (core 1, DMA, resampling, noise shaping) |
| `src/boom_test.c` | Bluetooth speaker test: Pico → one or two speakers, test tones |
| `src/main.c` | AUX test pattern and noise-hunting experiments |
| `config/btstack_config.h` | BTstack configuration |
| `build.sh` | Build / flash helper |

## License

MIT — see [LICENSE](LICENSE). This covers the project's own code. The Pico SDK
and BTstack keep their own licenses, and the parts of `src/boom_test.c` and
`src/hub.c` adapted from BTstack's examples remain subject to BTstack's terms
(see Credits).

## Credits

Built on the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
and [BTstack](https://github.com/bluekitchen/btstack) (Bluetooth stack,
including the bluedroid SBC codec). Parts of the A2DP handling are adapted
from BTstack's `a2dp_source_demo.c` and `a2dp_sink_demo.c`; BTstack's own
license terms apply to that code. BTstack is licensed for use with Raspberry
Pi's RP2040/RP2350 chips through the Pico SDK.
