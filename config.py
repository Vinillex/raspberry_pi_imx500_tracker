"""
Central configuration for the rpi_ai project.

Everything tunable lives here so the other modules stay logic-only.
Import as:  from config import KP, CH_ROLL, ...
"""

# --------------------------------------------------------------------------
# Serial ports (Raspberry Pi 5)
# --------------------------------------------------------------------------
# NOTE: on a Pi 5 the GPIO14/15 UART is /dev/ttyAMA0.
# /dev/serial0 points at ttyAMA10, the separate debug-header UART.
PORT_UP = "/dev/ttyAMA0"      # from ELRS receiver
PORT_DOWN = "/dev/ttyAMA2"    # to flight controller R2 pad (GPIO4/5)
BAUD = 420000                 # fixed by ELRS / Betaflight CRSF

SERIAL_READ_TIMEOUT = 0.02          # s; pyserial read() timeout on both ports
STOP_GRACE_S = 0.05                 # s; grace period in CrsfBridge.stop()
                                     # before closing the ports
FORWARD_ERROR_BACKOFF_S = 0.05      # s; retry delay after an error on the
                                     # receiver->FC pump
TELEMETRY_ERROR_BACKOFF_S = 0.01    # s; retry delay after an error on the
                                     # FC->receiver telemetry pump

# --------------------------------------------------------------------------
# CRSF protocol
# --------------------------------------------------------------------------
CRSF_SYNC = 0xC8
TYPE_RC_CHANNELS = 0x16
TYPE_LINK_STATS = 0x14

CRSF_MIN = 172
CRSF_MID = 992
CRSF_MAX = 1811

# --------------------------------------------------------------------------
# Channel map - AETR, confirmed on this airframe
#   CH1 Roll  CH2 Pitch  CH3 Throttle  CH4 Yaw
#   CH5 Aux1  CH6 Aux2   CH7 Aux3      CH8 Aux4
#   CH9 Aux5  CH10 Aux6
# Indices below are ZERO-BASED: channels[0] is CH1.
# --------------------------------------------------------------------------
CH_ROLL = 0
CH_PITCH = 1
CH_THROTTLE = 2
CH_YAW = 3
CH_AUX1 = 4    # repurposed as the arm channel - see ARM_CH_MIN below;
               # high exactly while ARMED, low otherwise (static-testing:
               # lowering Aux1 disarms) - see TrackController.apply();
               # never a raw passthrough of the pilot's switch
CH_AUX2 = 5    # static-testing: 3-pos gain selector for the ROLL axis
               # (low/mid/high -> Kp/Ki/Kd); also passes through to the FC
CH_AUX3 = 6    # static-testing: 3-pos gain selector for the PITCH axis
               # (low/mid/high -> Kp/Ki/Kd); also passes through to the FC
CH_AUX4 = 7    # static-testing: axis selector (low -> roll, high -> pitch)
               # for the gain tuner; also passes through to the FC
CH_AUX5 = 8    # repurposed as the detection-lock switch - see LOCK_CH_MIN below;
               # never forwarded to the FC (controller.py neutralises it)
CH_AUX6 = 9    # static-testing: scroll wheel (relative) that moves the
               # selected PID gain up/down; also passes through to the FC

CH_NAMES = ["Roll", "Pitch", "Thr", "Yaw",
            "Aux1", "Aux2", "Aux3", "Aux4",
            "Aux5", "Aux6",
            "CH11", "CH12", "CH13", "CH14", "CH15", "CH16"]

# --------------------------------------------------------------------------
# Control - tracking (PID roll/pitch override) runs ONLY in the ARMED
# state (armed + a fresh locked target) - never in SEARCHING (armed,
# target lost). See controller.py. There is no manual enable channel any
# more: ARMED alone is the gate.
# --------------------------------------------------------------------------
MAX_DEFLECTION = CRSF_MAX - CRSF_MID   # 819 - roll/pitch PID output may use
                                       # the full stick range once armed;
                                       # clamp_channel still bounds the wire

# Currently UNUSED by controller.py - throttle is a raw pilot passthrough
# in every state while arming itself is being bench-verified in isolation
# (see TrackController's docstring). Kept here, not deleted, for when
# throttle automation is reintroduced once arming is solid.
THROTTLE_ARMED = CRSF_MAX                              # ARMED, actively tracking
THROTTLE_SEARCHING = int(CRSF_MIN + 0.8 * (CRSF_MAX - CRSF_MIN))  # SEARCHING - 80%,
                                                        # no tracking

ROLL_KP = 620.0                 # proportional gain (counts per unit error)
ROLL_KI = 0.0                   # integral gain - starts at 0, windup-prone,
                                 # needs careful bench tuning
ROLL_KD = 35.0                   # derivative gain - bench-tuned after Kp
ROLL_I_MAX = 200.0              # anti-windup clamp on the integral accumulator

PITCH_KP = 260.0
PITCH_KI = 0.0
PITCH_KD = 0.0
PITCH_I_MAX = 200.0

DEADZONE = 0.06                # normalised error below which nothing is done
VISION_TIMEOUT = 0.25          # s; stale/absent target error is discarded
RC_TIMEOUT = 0.5               # s; no RC frames in this window means the receiver is gone
RATE_ALPHA = 0.35              # EMA smoothing on the derivative term
FPS_ALPHA = 0.1                # EMA smoothing on the displayed frame rate

ROLL_SIGN = +1                 # flip if corrections push the wrong way
PITCH_SIGN = +1

# --------------------------------------------------------------------------
# Live PID gain tuning (static-testing branch)
#   tracker.GainTuner + state.GainState + overlay._draw_gains
# --------------------------------------------------------------------------
# Bench workflow, all from the transmitter:
#   - Aux4 picks the axis:  low -> roll, high -> pitch
#   - Aux2 (roll) / Aux3 (pitch) are 3-position switches picking the gain:
#       low -> Kp, mid -> Ki, high -> Kd
#   - Aux6 is the scroll wheel, used as a RELATIVE control (only ARMED):
#     how far you turn it moves the selected gain (forward = up, back =
#     down); the value stays put wherever you stop and picks up from
#     there next time. Disarm to "commit", re-centre the wheel, re-arm to
#     keep going from the new value.
# Tuned values persist across arm/disarm cycles; restarting the script
# resets them to the ROLL_*/PITCH_* defaults above.
AUX_LOW_MAX = 700              # CRSF value at or below this = switch "low"
AUX_HIGH_MIN = 1300            # CRSF value at or above this = switch "high"
                               # (between the two = "mid", for Aux2/Aux3)
AUX6_DEADBAND = 0.06           # |wheel deflection|, as a fraction of full
                               # throw, below which the wheel counts as
                               # centred - the interlock that must be
                               # satisfied before Aux5 can lock

# Per-gain: how much one full Aux6 sweep (MIN->MAX) moves the gain.
# Halve for finer control, raise for coarser. Gains are floored at 0
# (turn the wheel back past 0 and it just stops) but have NO upper cap -
# Kp etc. can go as high as you want.
GAIN_SWEEP = {
    "roll_kp":  600.0,
    "roll_ki":  240.0,
    "roll_kd":  300.0,
    "pitch_kp": 600.0,
    "pitch_ki": 240.0,
    "pitch_kd": 300.0,
}

# --------------------------------------------------------------------------
# Vision
# --------------------------------------------------------------------------
MODEL = "/usr/share/imx500-models/imx500_network_nanodet_plus_416x416_pp.rpk"
TARGET_CLASS = 0
THRESHOLD = 0.5
MAX_DETECTIONS = 10
MAIN_SIZE = (640, 480)
HFLIP = 0
VFLIP = 0
MATCH_RADIUS_FRAC = 0.45      # of frame width; max frame-to-frame jump

LOCK_CH = CH_AUX5             # switch that locks onto the current detection
LOCK_CH_MIN = 1300             # must be at or above this to lock

ARM_CH = CH_AUX1              # switch that arms, but only edge-triggered
ARM_CH_MIN = 1300              # must be at or above this to count as "high"

# --------------------------------------------------------------------------
# Display colours (BGR)
# --------------------------------------------------------------------------
GREEN = (0, 255, 0)
ORANGE = (0, 165, 255)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
GRAY = (120, 120, 120)
YELLOW = (0, 255, 255)
BLACK = (0, 0, 0)
