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
               # only ever forced high by us when the ARMED latch fires,
               # never a raw passthrough of the pilot's switch
CH_AUX2 = 5
CH_AUX3 = 6
CH_AUX4 = 7    # repurposed as the GPS-rescue trigger - see RESCUE_CH_MIN below;
               # only ever forced high by us once GPS_RESCUE latches,
               # never a raw passthrough of the pilot's switch
CH_AUX5 = 8    # repurposed as the detection-lock switch - see LOCK_CH_MIN below;
               # never forwarded to the FC (controller.py neutralises it)
CH_AUX6 = 9    # repurposed as the camera zoom control - see ZOOM_MIN/MAX below;
               # never forwarded to the FC (controller.py neutralises it)

CH_NAMES = ["Roll", "Pitch", "Thr", "Yaw",
            "Aux1", "Aux2", "Aux3", "Aux4",
            "Aux5", "Aux6",
            "CH11", "CH12", "CH13", "CH14", "CH15", "CH16"]

# --------------------------------------------------------------------------
# Control - tracking (PID roll/pitch override) runs ONLY in the ARMED
# state (armed + a fresh locked target) - never in SEARCHING or
# GPS_RESCUE, even though both can only happen while armed=True. See
# controller.py. There is no manual enable channel any more: ARMED
# alone is the gate.
# --------------------------------------------------------------------------
MAX_DEFLECTION = 700           # max counts roll/pitch may sit away from
                                # CRSF_MID once armed (hard output clamp)

# Currently UNUSED by controller.py - throttle is a raw pilot passthrough
# in every state while arming itself is being bench-verified in isolation
# (see TrackController's docstring). Kept here, not deleted, for when
# throttle automation is reintroduced once arming is solid.
THROTTLE_ARMED = CRSF_MAX                              # ARMED, actively tracking
THROTTLE_SEARCHING = int(CRSF_MIN + 0.8 * (CRSF_MAX - CRSF_MIN))  # SEARCHING - 80%,
                                                        # no tracking

ROLL_KP = 260.0                 # proportional gain (counts per unit error)
ROLL_KI = 0.0                   # integral gain - starts at 0, windup-prone,
                                 # needs careful bench tuning
ROLL_KD = 90.0                  # derivative gain - required for stability
ROLL_I_MAX = 200.0              # anti-windup clamp on the integral accumulator

PITCH_KP = 260.0
PITCH_KI = 0.0
PITCH_KD = 90.0
PITCH_I_MAX = 200.0

DEADZONE = 0.06                # normalised error below which nothing is done
VISION_TIMEOUT = 0.25          # s; stale/absent target error is discarded
RC_TIMEOUT = 0.5               # s; no RC frames in this window means the receiver is gone
RATE_ALPHA = 0.35              # EMA smoothing on the derivative term
FPS_ALPHA = 0.1                # EMA smoothing on the displayed frame rate

ROLL_SIGN = +1                 # flip if corrections push the wrong way
PITCH_SIGN = +1

# --------------------------------------------------------------------------
# Vision
# --------------------------------------------------------------------------
MODEL = "/usr/share/imx500-models/imx500_network_nanodet_plus_416x416_pp.rpk"
TARGET_CLASS = 0
THRESHOLD = 0.5
MAX_DETECTIONS = 10
MAIN_SIZE = (640, 480)
HFLIP = 1
VFLIP = 1
MATCH_RADIUS_FRAC = 0.45      # of frame width; max frame-to-frame jump

ZOOM_MIN = 1.0                 # Aux6 low  -> no digital zoom (full FOV)
ZOOM_MAX = 4.0                 # Aux6 high -> max digital zoom

TARGET_BOX_FRAC = 0.5          # ARMED auto-zoom target: box height / frame height
AUTO_ZOOM_DEADBAND = 0.10       # +/-10% (40-60%) counts as close enough, no correction
AUTO_ZOOM_KP = 1.5              # proportional gain for the auto-zoom loop -
                                # needs bench tuning like KP/KD above
AUTO_ZOOM_MAX_STEP = 0.02       # max zoom-factor change per frame - slow ease
                                # in/out, so it doesn't overshoot
AUTO_ZOOM_EDGE_MARGIN_FRAC = 0.08   # if the box is within this fraction of any
                                     # frame edge, hold zoom - the crop is always
                                     # centred on the frame, so zooming further
                                     # risks cropping the target out entirely

LOCK_CH = CH_AUX5             # switch that locks onto the current detection
LOCK_CH_MIN = 1300             # must be at or above this to lock

ARM_CH = CH_AUX1              # switch that arms, but only edge-triggered
ARM_CH_MIN = 1300              # must be at or above this to count as "high"

GPS_RESCUE_TIMEOUT = 5.0       # s of continuous SEARCHING (while ARMED) before
                                # GPS_RESCUE triggers
RESCUE_CH = CH_AUX4            # manual GPS-rescue override - level-triggered,
                                # but only takes effect while already ARMED
                                # (see main_ai.py); the timeout path above is
                                # the only trigger that works before arming
RESCUE_CH_MIN = 1300            # must be at or above this to count as "high"

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
