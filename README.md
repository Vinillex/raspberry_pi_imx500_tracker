# rpi_ai — module layout

Raspberry Pi 5 sits between an ELRS receiver and a SpeedyBee F405 V4,
decoding CRSF, optionally modifying channels, and forwarding to the FC.

## Hardware

| Link | Pi pin | Detail |
|---|---|---|
| Gemini RX TX → Pi | GPIO15 / pin 10 | `/dev/ttyAMA0` @ 420000 |
| Pi → FC R2 pad | GPIO4 / pin 7 (TXD2) | `/dev/ttyAMA2` @ 420000 |
| FC T2 → Pi | GPIO5 / pin 29 (RXD2) | telemetry, optional |
| Ground | any GND | required |

Known-bad: **GPIO0** (pin 27) — low-side driver damaged, never use as UART TX.
`/dev/serial0` is `ttyAMA10`, the debug header — not the GPIO14/15 UART.

`/boot/firmware/config.txt` needs `dtparam=uart0=on` and `dtoverlay=uart2-pi5`.

## Files

| File | Responsibility | Hardware needed |
|---|---|---|
| `config.py` | every tunable constant, channel map | no |
| `crsf_protocol.py` | CRC, pack/unpack, frame parsing | no |
| `state.py` | `TargetState`, `ChannelState`, `ArmState`, `RescueState` (all thread-safe), `Stats` | no |
| `controller.py` | PID control law + safety gates | no |
| `tracker.py` | `AuxLock` (detection lock), `ArmLatch`, `GpsRescueLatch`, `ErrorTracker` (error + rate) | no |
| `vision.py` | IMX500 / picamera2 wrapper, auto-zoom | camera |
| `bridge.py` | serial ports and forwarding threads | serial |
| `overlay.py` | all OpenCV drawing | cv2 |
| `main_bridge.py` | entry: plain pass-through bridge | serial |
| `main_ai.py` | entry: bridge + vision + control | both |
| `test_protocol.py` | unit tests for the protocol layer | no |

Dependency direction is one-way:

```
config
  ├── crsf_protocol ── bridge ────────┐
  ├── crsf_protocol ── controller ────┤
  ├── state ── bridge, controller ────┤
  ├── vision ── tracker ──────────────┼── main_ai
  └── overlay ─────────────────────────┘
```

Nothing imports upward, so `crsf_protocol`, `controller` and `tracker` can
all be exercised on a laptop with no Pi attached.

## Running

```bash
source ~/crsf_env/bin/activate      # needs --system-site-packages

python3 test_protocol.py            # verify protocol layer, no hardware
python3 main_bridge.py --labels     # serial path only
python3 main_ai.py                  # full stack
python3 main_ai.py --no-display     # headless
```

## Channel map (AETR, confirmed)

| CH | Function | Index |
|---|---|---|
| 1 | Roll | 0 |
| 2 | Pitch | 1 |
| 3 | Throttle | 2 |
| 4 | Yaw | 3 |
| 5–8 | Aux1–4 | 4–7 |
| 9–10 | Aux5–6 | 8–9 |

All 16 channels are always decoded and re-encoded; `--show` only affects
what gets printed.

Aux5 (lock), Aux1 (arm), Aux4 (GPS rescue) and Aux6 (zoom) are all
repurposed as Pi-side controls — see `config.py`'s comments on each — and
none of their raw values ever reach the FC unmodified.

## Safety gates

`TrackController.apply()` in `controller.py`:

- Aux5/Aux6 (lock/zoom) are always neutralised — never forwarded to the FC.
- Aux1 (arm) and Aux4 (GPS rescue) are never a raw passthrough of the pilot's
  switch — their output is entirely decided by `tracker.ArmLatch` /
  `tracker.GpsRescueLatch`.
- **Not armed:** the pilot has full manual control of every channel.
- **Armed and actively tracking** (a fresh, locked target, not in GPS rescue):
  roll/pitch are **fully replaced** by two independently-tuned PID loops
  (not added to the pilot's stick input — the sticks have zero effect on
  those two axes), clamped to ±`MAX_DEFLECTION`.
- **Armed, SEARCHING** (target lost): roll/pitch go neutral.
- **GPS_RESCUE** (latched): roll/pitch go neutral, so the FC's own GPS
  Rescue flight mode (engaged via Aux4/CH8) can take over navigation.

**Throttle is currently left as a raw pilot passthrough in every state** —
armed or not, tracking, searching, or rescued. This is a deliberate,
temporary simplification while the arm/interlock logic itself is being
bench-verified: arming should not also have to fight Betaflight's
throttle-based arming checks (`min_check`) at the same time. The pilot
controls throttle manually via the stick throughout.

Arming requires a target to already be `LOCKED` (via Aux5/`AuxLock`), then a
low→high edge on Aux1 while still locked (`tracker.ArmLatch`). **Arming is a
one-way latch** — nothing in software disarms it once triggered, not Aux1
dropping, not losing the lock. GPS_RESCUE is the same: it latches
permanently on either 5s of continuous SEARCHING or Aux4 going high while
armed, and nothing clears it afterward. The only way back to manual control
is stopping the script (Ctrl+C).

## Bench test before flying

Props off, battery out, FC on USB. In Betaflight's Receiver tab:

- Not armed → CH1/CH2 mirror the sticks exactly; CH3 always mirrors the
  throttle stick, in every state below too; CH5 (arm) and CH8 (rescue) sit
  low regardless of switch position; CH9/CH10 (lock/zoom) always sit
  centred.
- Lock onto a target (Aux5), then raise Aux1 → CH5 snaps high, and CH1/CH2
  stop following the sticks entirely, driven by the PID instead (CH3 keeps
  following the throttle stick, unchanged). The correction should be
  *corrective* (step right, the bars should move the way that re-centres
  you) — backwards means flip `ROLL_SIGN` / `PITCH_SIGN` in `config.py`.
- Keep throttle at idle on your own while testing this — the FC still
  needs to see idle throttle to arm at all; this project just isn't the
  one enforcing it right now.
- This is a **one-way transition** — lowering Aux1, Aux5, or anything else
  will not undo it once armed.
- Losing the target (SEARCHING) → CH1/CH2 recentre.
- 5s of continuous SEARCHING, or raising Aux4 while armed → GPS_RESCUE: CH8
  snaps high, CH1/CH2 centre.
- Ctrl+C the script → bars go to failsafe, never hold a stale correction.
