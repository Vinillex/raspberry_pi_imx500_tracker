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
| `state.py` | `TargetState`, `ChannelState`, `ArmState`, `DisableState` (all thread-safe), `Stats` | no |
| `controller.py` | PID control law + safety gates | no |
| `tracker.py` | `AuxLock` (detection lock), `ArmLatch`, `DisableLatch`, `ErrorTracker` (error + rate) | no |
| `vision.py` | IMX500 / picamera2 wrapper, detection parsing | camera |
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
| 5 | Aux1 (arm) | 4 |
| 6–8 | Aux2–4 (free) | 5–7 |
| 9 | Aux5 (lock) | 8 |
| 10 | Aux6 (free) | 9 |

All 16 channels are always decoded and re-encoded; `--show` only affects
what gets printed.

Only **Aux1 (arm)** and **Aux5 (lock)** are repurposed as Pi-side controls
— see `config.py`'s comments — and their raw values never reach the FC
unmodified. **Aux2, Aux3, Aux4 and Aux6 are free channels** and pass
straight through untouched (Aux4 was the GPS-rescue trigger and Aux6 the
camera zoom; both features have been removed).

## Safety gates

`TrackController.apply()` in `controller.py`:

- Aux5 (lock) is always neutralised — never forwarded to the FC.
  Aux2/Aux3/Aux4/Aux6 pass straight through (free channels).
- Aux1 (arm) is never a raw passthrough of the pilot's switch.
- **CH5 (Aux1) is high whenever ARMED *or* LOCKED**, not only once actually
  armed. Before arming, CH5 mirrors `LOCKED` live: high while a target is
  currently locked (Aux5) with a fresh measurement, low the instant it isn't
  — a bench-testing convenience, since unlike `ArmLatch`, `AuxLock`'s lock
  isn't a one-way latch, so toggling Aux5 gives a freely retestable CH5
  signal without restarting the script. **This means the FC receives an arm
  request the moment a lock is acquired, independent of the pilot's own
  Aux1 switch position.** Once `ArmLatch` actually fires (see below), CH5
  stays high forever regardless of `LOCKED` afterward.
- **Not armed:** the pilot has full manual control of every channel.
- **Armed and actively tracking** (a fresh, locked target):
  roll/pitch are **fully replaced** by two independently-tuned PID loops
  (not added to the pilot's stick input — the sticks have zero effect on
  those two axes), clamped to ±`MAX_DEFLECTION`.
- **Armed, SEARCHING** (target lost): roll/pitch go neutral.

**Throttle is currently left as a raw pilot passthrough in every state** —
armed or not, tracking, searching. This is a deliberate, temporary
simplification while the arm/interlock logic itself is being bench-verified:
arming should not also have to fight Betaflight's throttle-based arming
checks (`min_check`) at the same time. The pilot controls throttle manually
via the stick throughout.

Arming (the actual `armed` software state that gates PID tracking) requires
a target to already be `LOCKED` (via Aux5/`AuxLock`), then a low→high edge
on the **pilot's own** Aux1 input while still locked (`tracker.ArmLatch`) —
this software-side detection reads the pilot's raw switch directly and is
unaffected by whatever CH5 currently outputs to the FC. But because CH5
already mirrors LOCKED (above), in the normal lock-then-arm sequence CH5
was typically already high by the time this official edge fires — so the
FC's *actual* arm attempt, and any refusal, most likely already happened
earlier, the moment the lock was acquired, not at this later software
event. **Arming is a one-way latch** — nothing in software disarms it once
triggered, not Aux1 dropping, not losing the lock. Short of stopping the
script (Ctrl+C), the only way back to manual control is the DISABLED kill
switch below.

### DISABLED — a bench-testing kill switch

Lowering Aux1 while ARMED triggers **DISABLED** (`tracker.DisableLatch`):
CH5 is forced low — disarming the FC — and `TrackController.apply()` stops
doing anything else at all for the rest of the run, overriding
ARMED/LOCKED and everything downstream of them. The overlay shows
**DISABLED** in black, with no box or blocking-state labels. Like every
other latch here, **DISABLED is one-way** — raising Aux1 again does
nothing; only restarting `main_ai.py` clears it.

This exists purely for bench testing (an actual, deliberate abort switch,
as opposed to `ArmLatch`'s intentional refusal to disarm on an accidental
blip), and is separate from `disable_state` simply not being wired up
(`TrackController(..., disable_state=None)`, the default) — in that case
DISABLED can never trigger at all, identical to before this existed.

## Bench test before flying

Props off. **Power the FC from the battery, not USB** — confirmed on this
exact setup: a live USB/Configurator connection blocks arming outright
(the `MSP` arming-disable flag), independent of anything this project does.
Verify arming by ear/eye (motor beep, status LED) or a radio telemetry
screen, not by watching Betaflight's Receiver tab live.

- Not armed, nothing locked → CH1/CH2 mirror the sticks exactly; CH3 always
  mirrors the throttle stick, in every state below too; CH5 (arm) sits low;
  CH9 (lock) always sits centred; CH6/CH7/CH8/CH10 (Aux2/3/4/6, free) pass
  straight through.
- Lock onto a target (Aux5) → **CH5 goes high immediately, before you've
  touched Aux1 at all.** This is deliberate (see Safety gates above) — the
  FC will attempt to arm right here if throttle is at idle and nothing else
  blocks it. Lose the lock → CH5 drops back low; this part is freely
  repeatable, not one-way.
- With the target still locked, raise Aux1 → the *software* records an
  official arm (`armed` latches true, gates PID tracking), but CH5 itself
  may show no new transition on the wire, since it was likely already high
  from the lock. CH1/CH2 stop following the sticks entirely, driven by the
  PID instead (CH3 keeps following the throttle stick, unchanged). The
  correction should be *corrective* (step right, the bars should move the
  way that re-centres you) — backwards means flip `ROLL_SIGN` / `PITCH_SIGN`
  in `config.py`.
- Keep throttle at idle on your own while testing this — the FC still
  needs to see idle throttle to arm at all; this project just isn't the
  one enforcing it right now.
- Once `armed` latches, CH5 stays high for good — this part **is** a
  **one-way transition**: lowering Aux1, Aux5, or anything else will not
  undo it.
- Losing the target (SEARCHING) → CH1/CH2 recentre.
- Lower Aux1 while ARMED → **DISABLED**: CH5 snaps low (this actually
  disarms the FC), overlay shows "DISABLED" in black, no box. This is also
  a **one-way transition** — raising Aux1 again does not undo it; only
  restarting the script does.
- Ctrl+C the script → bars go to failsafe, never hold a stale correction.
