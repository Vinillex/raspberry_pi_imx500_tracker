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
| `state.py` | `TargetState` (thread-safe), `Stats` | no |
| `controller.py` | PD control law + safety gates | no |
| `tracker.py` | selection/lock state machine, error + rate | no |
| `vision.py` | IMX500 / picamera2 wrapper | camera |
| `bridge.py` | serial ports and forwarding threads | serial |
| `overlay.py` | all OpenCV drawing | cv2 |
| `main_bridge.py` | entry: plain pass-through bridge | serial |
| `main_ai.py` | entry: bridge + vision + control | both |
| `test_protocol.py` | unit tests for the protocol layer | no |

Dependency direction is one-way:

```
config
  └── crsf_protocol ── bridge ──┐
  └── state ── controller ──────┼── main_ai
  └── vision ── tracker ────────┤
                overlay ────────┘
```

Nothing imports upward, so `crsf_protocol`, `controller` and `tracker` can
all be exercised on a laptop with no Pi attached.

## Running

```bash
source ~/crsf_env/bin/activate      # needs --system-site-packages

python3 test_protocol.py            # verify protocol layer, no hardware
python3 main_bridge.py --labels     # serial path only
python3 main_ai.py                  # full stack
python3 main_ai.py --axis yaw       # horizontal error drives yaw
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

## Safety gates

`TrackController.apply()` returns channels unchanged unless all pass:

1. `AI_ENABLE_CH` (CH6) above `AI_ENABLE_MIN`
2. vision data newer than `VISION_TIMEOUT`
3. a target is locked
4. output clamped to ±`MAX_AUTHORITY`

Corrections are **added** to pilot stick input, never substituted, so the
pilot can always override. Throttle and aux channels are never touched.

## Bench test before flying

Props off, battery out, FC on USB. In Betaflight's Receiver tab:

- AI switch off → bars mirror sticks exactly
- AI switch on, target locked → CH1/CH2 nudge, and the nudge should be
  *corrective* (step right, bar should move the way that re-centres you).
  Backwards means flip `ROLL_SIGN` / `PITCH_SIGN` in `config.py`.
- AI switch off mid-track → bars snap back to pure stick input
- Ctrl+C the script → bars go to failsafe, never hold a stale correction