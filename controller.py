"""
Control law and safety gates.

No serial, no camera, no threads - takes a channel list and target state,
returns a channel list. That makes it testable without hardware.
"""

import time

from config import (AI_ENABLE_CH, AI_ENABLE_MIN, MAX_AUTHORITY, KP, KD,
                    DEADZONE, VISION_TIMEOUT, ROLL_SIGN, PITCH_SIGN,
                    CH_ROLL, CH_PITCH, CH_YAW, CH_AUX1, CH_AUX4, CH_AUX5,
                    CH_AUX6, CRSF_MIN, CRSF_MID, CRSF_MAX)
from crsf_protocol import clamp_channel

REPURPOSED_CHANNELS = (CH_AUX5, CH_AUX6)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TrackController:
    """
    Adds a bounded correction to the pilot's stick input.

    The correction is ADDED to what the pilot is already commanding - it
    never replaces it. The pilot can always override by moving the stick,
    and can always cut the AI out entirely with the enable switch.
    """

    def __init__(self, target_state, arm_state, rescue_state, horiz_axis="roll"):
        self.target = target_state
        self.arm_state = arm_state
        self.rescue_state = rescue_state
        self.horiz_idx = CH_YAW if horiz_axis == "yaw" else CH_ROLL

    def apply(self, channels):
        """
        Returns the (possibly modified) channel list.

        Four gates, all of which must pass:
          1. pilot's enable switch is high
          2. vision data is fresh
          3. a target is locked
          4. output is clamped to MAX_AUTHORITY
        """
        self.target.ai_engaged = False
        self.target.last_horiz_corr = 0
        self.target.last_vert_corr = 0

        # Aux5 (detection lock) and Aux6 (camera zoom) are repurposed as
        # Pi-side controls (see vision.py / tracker.py / main_ai.py) and
        # must never reach the FC - neutralise them unconditionally,
        # regardless of the gates below.
        for aux_ch in REPURPOSED_CHANNELS:
            if len(channels) > aux_ch:
                channels[aux_ch] = CRSF_MID

        # Aux1/CH5 is the arm channel forwarded to the FC - it has NO
        # direct connection to the pilot's raw Aux1 value at all, armed
        # or not. The output is entirely decided by the ARMED latch
        # (tracker.ArmLatch, driven from main_ai.py, which requires the
        # LOCKED-first interlock): high only while armed, low otherwise.
        if len(channels) > CH_AUX1:
            channels[CH_AUX1] = CRSF_MAX if self.arm_state.get() else CRSF_MIN

        # Aux4/CH8 is the GPS-rescue channel forwarded to the FC - same
        # treatment as CH5: fully decided by the software latch
        # (tracker.GpsRescueLatch, driven from main_ai.py), never a raw
        # passthrough of the pilot's Aux4 switch.
        if len(channels) > CH_AUX4:
            channels[CH_AUX4] = CRSF_MAX if self.rescue_state.get() else CRSF_MIN

        # Gate 1 - pilot's enable switch
        if len(channels) <= AI_ENABLE_CH:
            return channels
        if channels[AI_ENABLE_CH] < AI_ENABLE_MIN:
            return channels

        ex, ey, ex_rate, ey_rate, locked, stamp = self.target.snapshot()

        # Gate 2 - freshness. A stalled vision loop must not hold a
        # stale correction on the sticks.
        if time.monotonic() - stamp > VISION_TIMEOUT:
            return channels

        # Gate 3 - a target actually exists
        if not locked:
            return channels

        # PD. The derivative term is not optional: stick deflection commands
        # acceleration, so proportional-only control on a position error
        # behaves as a double integrator and will oscillate.
        horiz = 0.0
        vert = 0.0
        if abs(ex) > DEADZONE:
            horiz = ROLL_SIGN * (KP * ex + KD * ex_rate)
        if abs(ey) > DEADZONE:
            vert = PITCH_SIGN * (KP * ey + KD * ey_rate)

        # Gate 4 - hard authority limit
        horiz = clamp(horiz, -MAX_AUTHORITY, MAX_AUTHORITY)
        vert = clamp(vert, -MAX_AUTHORITY, MAX_AUTHORITY)

        channels[self.horiz_idx] = clamp_channel(
            channels[self.horiz_idx] + horiz)
        channels[CH_PITCH] = clamp_channel(channels[CH_PITCH] + vert)

        # Throttle and remaining aux channels pass through untouched
        # (Aux1, Aux4, Aux5 and Aux6 were already handled above).

        self.target.ai_engaged = True
        self.target.last_horiz_corr = int(horiz)
        self.target.last_vert_corr = int(vert)
        return channels


class PassThrough:
    """Null controller - forwards channels unchanged. Useful for testing
    the serial path in isolation from the control law."""

    def apply(self, channels):
        return channels
