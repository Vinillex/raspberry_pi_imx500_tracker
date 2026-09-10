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
| `state.py` | `TargetState`, `ChannelState`, `ArmState`, `GainState` (all thread-safe), `Stats` | no |
| `controller.py` | PID control law + safety gates | no |
| `tracker.py` | `AuxLock` (detection lock), `ArmLatch`, `GainTuner` (live PID tuning), `ErrorTracker` (error + rate) | no |
| `vision.py` | IMX500 / picamera2 wrapper, detection parsing | camera |
| `bridge.py` | serial ports and forwarding threads | serial |
| `overlay.py` | all OpenCV drawing | cv2 |
| `main_bridge.py` | entry: plain pass-through bridge | serial |
| `main_ai.py` | entry: bridge + vision + control | both |
| `test_protocol.py` | unit tests for the protocol layer | no |
| `test_tracker.py` | unit tests for lock / arm / gain tuner / error tracking | no |
| `test_controller.py` | unit tests for the PID + safety gates | no |

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
python3 test_tracker.py             # lock / arm / gain tuner, no hardware
python3 test_controller.py          # PID + safety gates, no hardware
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
| 6 | Aux2 (roll gain select, 3-pos) | 5 |
| 7 | Aux3 (pitch gain select, 3-pos) | 6 |
| 8 | Aux4 (tuner axis select) | 7 |
| 9 | Aux5 (lock) | 8 |
| 10 | Aux6 (gain scroll wheel) | 9 |

All 16 channels are always decoded and re-encoded; `--show` only affects
what gets printed.

Only **Aux1 (arm)** and **Aux5 (lock)** are repurposed as Pi-side controls
— see `config.py`'s comments — and their raw values never reach the FC
unmodified. **Aux2, Aux3, Aux4 and Aux6** are read by the live PID-gain
tuner (see below) but the controller still passes them straight through
to the FC untouched.

## Safety gates

> **This is the `static-testing` branch.** Arming is deliberately
> repeatable here (lower Aux1 → disarm → re-arm) so PID gains can be
> dialled in over many short arm cycles. On `main`, arming is one-way and
> lowering Aux1 while armed triggers a terminal DISABLED kill switch.

`TrackController.apply()` in `controller.py`:

- Aux5 (lock) is always neutralised — never forwarded to the FC.
  Aux2/Aux3/Aux4/Aux6 pass straight through (the gain tuner reads them
  but the controller doesn't touch them).
- Aux1 (arm) is never a raw passthrough of the pilot's switch.
- **CH5 (Aux1) is high exactly while ARMED, low otherwise.** Since
  `ArmLatch` needs a `LOCKED` target before the pilot's Aux1 edge counts,
  and lowering Aux1 disarms again, CH5 tracks the pilot's Aux1 one-for-one
  once a lock exists — up to arm the FC, down to disarm it.
- **Not armed:** the pilot has full manual control of every channel.
- **Armed and actively tracking** (a fresh, locked target):
  roll/pitch are **fully replaced** by two independently-tuned PID loops
  (not added to the pilot's stick input — the sticks have zero effect on
  those two axes), clamped to ±`MAX_DEFLECTION`. The PID gains are read
  live from `GainState` every frame (see the tuner section below).
- **Armed, SEARCHING** (target lost): roll/pitch go neutral.

**Throttle is left as a raw pilot passthrough in every state** — armed or
not, tracking, searching. The pilot controls throttle manually via the
stick throughout; keep it at idle so the FC will arm.

Arming (the `armed` software state that gates PID tracking) requires a
target to already be `LOCKED` (via Aux5/`AuxLock`), then a low→high edge
on the **pilot's own** Aux1 input while still locked (`tracker.ArmLatch`).
**Lowering Aux1 disarms** — `armed` clears, CH5 drops, the FC disarms, the
PID integrators reset, and you're back to manual control ready to select
and tune the next gain. Losing the lock while Aux1 stays high does *not*
disarm (that's SEARCHING).

### Live PID gain tuning

`tracker.GainTuner` (main thread) → `state.GainState` → `controller` (bridge
thread). Six gains — Kp/Ki/Kd for roll and for pitch — start at the
`ROLL_*`/`PITCH_*` defaults in `config.py` and are adjusted from the
transmitter:

| Gain | Switches |
|---|---|
| Kp Roll  | Aux2 low,  Aux4 low  |
| Ki Roll  | Aux2 mid,  Aux4 low  |
| Kd Roll  | Aux2 high, Aux4 low  |
| Kp Pitch | Aux3 low,  Aux4 high |
| Ki Pitch | Aux3 mid,  Aux4 high |
| Kd Pitch | Aux3 high, Aux4 high |

**Aux6 is a spring-return scroll wheel**: hold it forward to ramp the
selected gain up, back to ramp it down, release (centre) to hold. Per-gain
ranges and ramp rates are `GAIN_LIMITS` in `config.py`. Tuned values
persist across arm/disarm cycles; restarting `main_ai.py` resets them.

- **The wheel only adjusts while ARMED.** In DETECTING you use
  Aux2/Aux3/Aux4 to *pick* which gain you'll tune; Aux6 does nothing
  until you're armed and tracking.
- **Aux5 will not lock until Aux6 is centred** — an interlock so the
  wheel is neutral at the moment you arm (otherwise it would start
  ramping hard immediately). The overlay shows `CENTER AUX6 TO LOCK`
  across the frame centre while the wheel is off centre.
- Overlay: in **DETECTING** the top-right panel lists all six gains, the
  selected one flagged `>` in yellow. While **ARMED** it collapses to a
  single yellow line — the selected gain's name and its live value —
  top-right, so you can watch it move as you turn the wheel. Hidden in
  between (locked, not yet armed).

Typical loop: pick the gain (Aux2/3/4) → centre Aux6 → raise Aux5 to lock
→ raise Aux1 to arm → scroll Aux6 while watching the tracking response →
lower Aux1 to disarm → repeat for the next gain.

## Bench test before flying

Props off. **Power the FC from the battery, not USB** — confirmed on this
exact setup: a live USB/Configurator connection blocks arming outright
(the `MSP` arming-disable flag), independent of anything this project does.
Verify arming by ear/eye (motor beep, status LED) or a radio telemetry
screen, not by watching Betaflight's Receiver tab live.

- Not armed, nothing locked (DETECTING) → CH1/CH2 mirror the sticks
  exactly; CH3 always mirrors the throttle stick, in every state below
  too; CH5 (arm) sits low; CH9 (lock) always sits centred;
  CH6/CH7/CH8/CH10 (Aux2/3/4/6) pass straight through. The six live gains
  show top-right, the selected one flagged `>`.
- Move Aux6 off centre → `CENTER AUX6 TO LOCK` appears and Aux5 is
  ignored. Centre it again to proceed.
- Lock onto a target (Aux5, wheel centred) → box turns orange, `LOCKED`.
  CH5 stays **low** — locking alone does not arm on this branch.
- With the target still locked, raise Aux1 → `armed` latches, CH5 goes
  high (the FC arms if throttle is at idle), CH1/CH2 stop following the
  sticks and are driven by the PID (CH3 unchanged). The correction should
  be *corrective* (step right → bars move the way that re-centres you) —
  backwards means flip `ROLL_SIGN` / `PITCH_SIGN` in `config.py`.
- Now armed → the top-right readout collapses to the selected gain's
  live value. Scroll Aux6 forward/back → it ramps (the wheel is inert
  until this point); watch the value and the tracking response change.
- Losing the target (SEARCHING) → CH1/CH2 recentre; still armed.
- **Lower Aux1 → disarm**: CH5 snaps low (disarms the FC), CH1/CH2 return
  to the sticks, PID integrators reset. The gain keeps its new value.
  Select the next gain and repeat. This is repeatable, *not* one-way, on
  this branch.
- Ctrl+C the script → bars go to failsafe, never hold a stale correction.
