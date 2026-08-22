"""
Control law and safety gates.

No serial, no camera, no threads - takes a channel list and target state,
returns a channel list. That makes it testable without hardware.
"""

import time

from config import (ROLL_KP, ROLL_KI, ROLL_KD, ROLL_I_MAX,
                    PITCH_KP, PITCH_KI, PITCH_KD, PITCH_I_MAX,
                    MAX_DEFLECTION, DEADZONE, VISION_TIMEOUT,
                    ROLL_SIGN, PITCH_SIGN, CH_ROLL, CH_PITCH,
                    CH_AUX1, CH_AUX4, CH_AUX5, CH_AUX6,
                    CRSF_MIN, CRSF_MID, CRSF_MAX)
from crsf_protocol import clamp_channel

REPURPOSED_CHANNELS = (CH_AUX5, CH_AUX6)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _set_channel(channels, ch, value):
    """channels[ch] = value, if that index exists - a no-op otherwise
    for channel lists shorter than expected."""
    if len(channels) > ch:
        channels[ch] = value


class PID:
    """Simple PID with anti-windup clamping on the integral term.

    The derivative term takes an already-computed rate (see
    tracker.ErrorTracker) rather than differencing error itself, since
    the rate is measured at the vision loop's cadence and smoothed
    there - differencing again here, at the bridge thread's much
    higher call rate, would just amplify noise.
    """

    def __init__(self, kp, ki, kd, i_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max = i_max
        self._integral = 0.0

    def update(self, error, rate, dt):
        self._integral = clamp(self._integral + error * dt,
                               -self.i_max, self.i_max)
        return self.kp * error + self.ki * self._integral + self.kd * rate

    def reset(self):
        self._integral = 0.0


class TrackController:
    """
    Full autonomous override, but ONLY while actively tracking.

    Tracking - the PID roll/pitch override - runs exclusively in the
    ARMED state: armed AND a fresh, locked target. It does NOT run in
    SEARCHING (armed, target lost - roll/pitch go neutral) or
    GPS_RESCUE (rescue_state latched - roll/pitch neutral, letting the
    FC's own GPS Rescue flight mode, engaged via Aux4/CH8, take over
    navigation). LOCKED/DETECTED/etc. are pre-arm states and are
    already excluded by the top-level `armed` gate.

    Unlike a bumper-correction design, this does NOT add a bounded
    nudge on top of pilot input - whenever tracking is active, the
    pilot's roll/pitch sticks have zero effect. Two independently-tuned
    PID loops drive roll/pitch directly toward the locked target
    (TargetState, fed by tracker.ErrorTracker).

    Throttle is deliberately left as a raw pilot passthrough in every
    state (not armed, armed+tracking, SEARCHING, GPS_RESCUE) - the
    pilot controls it manually via the stick at all times. This is a
    temporary simplification while the arm/interlock logic itself is
    being bench-verified: arming should not also be fighting
    Betaflight's throttle-based arming checks (min_check) at the same
    time. Yaw and all channels not explicitly handled here pass
    straight through too.

    There is no manual "AI enable" channel - ARMED (tracker.ArmLatch,
    itself gated by LOCKED-first, see main_ai.py) is the sole top-level
    gate, and once armed there is no software path back to manual
    roll/pitch control short of restarting the script.

    CH5 (the arm channel forwarded to the FC) is high whenever ARMED OR
    LOCKED - see the CH5 comment in apply(). This is a bench-testing
    convenience: LOCKED (Aux5) is freely toggleable, unlike ARMED, so it
    gives a CH5 signal you can raise/lower repeatedly without restarting
    the script. It also means the FC gets an arm request the moment a
    lock is acquired, not only on the pilot's own Aux1 edge.

    DISABLED (tracker.DisableLatch, one-way, bench-testing kill switch -
    triggers when Aux1 reads low while ARMED/GPS_RESCUE) overrides all of
    the above: CH5/CH8 are forced low and nothing else in apply() runs,
    for the rest of this run.
    """

    def __init__(self, target_state, arm_state, rescue_state,
                disable_state=None):
        self.target = target_state
        self.arm_state = arm_state
        self.rescue_state = rescue_state
        self.disable_state = disable_state
        self.roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, ROLL_I_MAX)
        self.pitch_pid = PID(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_I_MAX)
        self._prev_t = time.monotonic()

    def apply(self, channels):
        """Returns the (possibly modified) channel list."""
        # Aux5 (detection lock) and Aux6 (camera zoom) are repurposed as
        # Pi-side controls (see vision.py / tracker.py / main_ai.py) and
        # must never reach the FC - neutralise them unconditionally.
        for aux_ch in REPURPOSED_CHANNELS:
            _set_channel(channels, aux_ch, CRSF_MID)

        # DISABLED - bench-test kill switch (tracker.DisableLatch, one-way,
        # driven from main_ai.py once Aux1 reads low while ARMED or
        # GPS_RESCUE). Overrides everything below: CH5/CH8 forced low
        # (disarms the FC) and nothing else in this function runs from
        # here on - full pilot passthrough on roll/pitch/throttle, exactly
        # like "not armed". disable_state=None (not wired up) means this
        # never triggers, same as if the feature didn't exist. Nothing
        # clears this short of restarting main_ai.py.
        if self.disable_state is not None and self.disable_state.get():
            _set_channel(channels, CH_AUX1, CRSF_MIN)
            _set_channel(channels, CH_AUX4, CRSF_MIN)
            self.roll_pid.reset()
            self.pitch_pid.reset()
            return channels

        armed = self.arm_state.get()
        now = time.monotonic()
        ex, ey, ex_rate, ey_rate, locked, stamp = self.target.snapshot()
        fresh = (now - stamp) <= VISION_TIMEOUT
        is_locked_now = locked and fresh

        # Aux1/CH5 is the arm channel forwarded to the FC - it has NO
        # direct connection to the pilot's raw Aux1 value at all. Before
        # ARMED, CH5 mirrors LOCKED live: high whenever a target is
        # currently locked (Aux5) with a fresh measurement, low the
        # instant it isn't - a bench-testing aid, since unlike ARMED,
        # LOCKED isn't a one-way latch, so toggling Aux5 gives a freely
        # retestable CH5 signal without restarting the script. NOTE: this
        # means the FC receives an arm request as soon as a lock is
        # acquired, independent of the pilot's own Aux1 switch position.
        # Once ARMED fires (tracker.ArmLatch, one-way, requires an Aux1
        # edge while already locked - see main_ai.py), CH5 stays high
        # forever regardless of LOCKED afterward, same as before.
        _set_channel(channels, CH_AUX1,
                    CRSF_MAX if (armed or is_locked_now) else CRSF_MIN)

        # Aux4/CH8 is the GPS-rescue channel forwarded to the FC - same
        # treatment as CH5: fully decided by the software latch
        # (tracker.GpsRescueLatch, driven from main_ai.py), never a raw
        # passthrough of the pilot's Aux4 switch.
        _set_channel(channels, CH_AUX4,
                    CRSF_MAX if self.rescue_state.get() else CRSF_MIN)

        dt = max(now - self._prev_t, 1e-3)
        self._prev_t = now

        if not armed:
            # Not armed - pilot has full manual control of roll/pitch/
            # throttle (CH5 above may still be high, if LOCKED). Keep the
            # PID integrators at zero so arming doesn't inherit stale
            # windup from an old attempt.
            self.roll_pid.reset()
            self.pitch_pid.reset()
            return channels

        rescue = self.rescue_state.get()
        tracking = is_locked_now and not rescue

        if not tracking:
            # SEARCHING (no/stale target) or GPS_RESCUE - no tracking
            # either way. Hold roll/pitch neutral rather than coast on a
            # stale/absent error, and don't let the integral wind up
            # against a signal that isn't there. Throttle is left alone
            # (raw pilot passthrough) - see the class docstring.
            self.roll_pid.reset()
            self.pitch_pid.reset()
            _set_channel(channels, CH_ROLL, CRSF_MID)
            _set_channel(channels, CH_PITCH, CRSF_MID)
            return channels

        # ARMED with a fresh, locked target, not in rescue - active
        # tracking. Throttle is still left alone (raw pilot passthrough).
        roll_out = 0.0
        if abs(ex) > DEADZONE:
            roll_out = ROLL_SIGN * self.roll_pid.update(ex, ex_rate, dt)
        pitch_out = 0.0
        if abs(ey) > DEADZONE:
            pitch_out = PITCH_SIGN * self.pitch_pid.update(ey, ey_rate, dt)

        roll_out = clamp(roll_out, -MAX_DEFLECTION, MAX_DEFLECTION)
        pitch_out = clamp(pitch_out, -MAX_DEFLECTION, MAX_DEFLECTION)

        # Full override - NOT added to pilot input. The remote's
        # roll/pitch sticks have zero effect from here on.
        channels[CH_ROLL] = clamp_channel(CRSF_MID + roll_out)
        channels[CH_PITCH] = clamp_channel(CRSF_MID + pitch_out)

        return channels


class PassThrough:
    """Null controller - forwards channels unchanged. Useful for testing
    the serial path in isolation from the control law."""

    def apply(self, channels):
        return channels
