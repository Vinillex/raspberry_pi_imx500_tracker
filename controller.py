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
                    CH_AUX1, CH_AUX5,
                    CRSF_MIN, CRSF_MID, CRSF_MAX)
from crsf_protocol import clamp_channel

# Aux5 (detection lock) is a Pi-side control and must never reach the FC.
# Aux6 used to be here too (camera zoom); zoom logic has been removed and
# Aux6 now passes straight through.
REPURPOSED_CHANNELS = (CH_AUX5,)


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
        # Only integrate while Ki is actually in use. Gains are live-tuned
        # (GainState), and Ki starts at 0 - without this, the accumulator
        # would wind up silently and then kick hard the instant Ki is
        # dialled up from the wheel.
        if self.ki:
            self._integral = clamp(self._integral + error * dt,
                                   -self.i_max, self.i_max)
        else:
            self._integral = 0.0
        return self.kp * error + self.ki * self._integral + self.kd * rate

    def reset(self):
        self._integral = 0.0


class TrackController:
    """
    Full autonomous override, but ONLY while actively tracking.

    Tracking - the PID roll/pitch override - runs exclusively in the
    ARMED state: armed AND a fresh, locked target. It does NOT run in
    SEARCHING (armed, target lost - roll/pitch go neutral).
    LOCKED/DETECTED/etc. are pre-arm states and are already excluded by
    the top-level `armed` gate.

    Unlike a bumper-correction design, this does NOT add a bounded
    nudge on top of pilot input - whenever tracking is active, the
    pilot's roll/pitch sticks have zero effect. Two independently-tuned
    PID loops drive roll/pitch directly toward the locked target
    (TargetState, fed by tracker.ErrorTracker).

    Throttle is deliberately left as a raw pilot passthrough in every
    state (not armed, armed+tracking, SEARCHING) - the pilot controls it
    manually via the stick at all times. This is a temporary
    simplification while the arm/interlock logic itself is being
    bench-verified: arming should not also be fighting Betaflight's
    throttle-based arming checks (min_check) at the same time. Yaw and
    all channels not explicitly handled here (Aux2/Aux3/Aux4/Aux6) pass
    straight through too.

    There is no manual "AI enable" channel - ARMED (tracker.ArmLatch,
    itself gated by LOCKED-first, see main_ai.py) is the sole top-level
    gate on roll/pitch tracking.

    CH5 (the arm channel forwarded to the FC) is high exactly while
    ARMED and low otherwise. On this static-testing branch ArmLatch is
    not one-way: lowering the pilot's Aux1 disarms, which drops CH5 and
    disarms the FC, ready for the next tuning pass.

    PID gains are read live from GainState every frame (fed by
    tracker.GainTuner in main_ai.py), so Kp/Ki/Kd can be dialled in from
    the transmitter while tracking. gain_state=None keeps the config
    defaults baked into the PIDs at construction.
    """

    def __init__(self, target_state, arm_state, gain_state=None):
        self.target = target_state
        self.arm_state = arm_state
        self.gain_state = gain_state
        self.roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, ROLL_I_MAX)
        self.pitch_pid = PID(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_I_MAX)
        self._prev_t = time.monotonic()

    def apply(self, channels):
        """Returns the (possibly modified) channel list."""
        # Aux5 (detection lock) is a Pi-side control (see tracker.py /
        # main_ai.py) and must never reach the FC - neutralise it.
        # Aux2/Aux3/Aux4/Aux6 are free channels and pass straight through.
        for aux_ch in REPURPOSED_CHANNELS:
            _set_channel(channels, aux_ch, CRSF_MID)

        armed = self.arm_state.get()
        now = time.monotonic()
        ex, ey, ex_rate, ey_rate, locked, stamp = self.target.snapshot()
        fresh = (now - stamp) <= VISION_TIMEOUT
        is_locked_now = locked and fresh

        # Aux1/CH5 is the arm channel forwarded to the FC - never a raw
        # passthrough of the pilot's switch. It is high exactly while
        # ARMED and low otherwise. ArmLatch (see main_ai.py) needs a
        # LOCKED target before the pilot's Aux1 edge counts, and on this
        # static-testing branch lowering Aux1 disarms again - so CH5
        # follows the pilot's Aux1 one-for-one once a lock exists,
        # dropping the moment they lower it, ready for the next pass.
        _set_channel(channels, CH_AUX1, CRSF_MAX if armed else CRSF_MIN)

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

        if not is_locked_now:
            # SEARCHING - armed with no fresh target. Hold roll/pitch
            # neutral rather than coast on a stale/absent error, and
            # don't let the integral wind up against a signal that isn't
            # there. Throttle is left alone (raw pilot passthrough) - see
            # the class docstring.
            self.roll_pid.reset()
            self.pitch_pid.reset()
            _set_channel(channels, CH_ROLL, CRSF_MID)
            _set_channel(channels, CH_PITCH, CRSF_MID)
            return channels

        # ARMED with a fresh, locked target - active tracking. Throttle
        # is still left alone (raw pilot passthrough). Pull the latest
        # Kp/Ki/Kd from the bench tuner just before using them, so a
        # wheel adjustment this frame takes effect this frame.
        if self.gain_state is not None:
            self.roll_pid.kp, self.roll_pid.ki, self.roll_pid.kd = \
                self.gain_state.roll()
            self.pitch_pid.kp, self.pitch_pid.ki, self.pitch_pid.kd = \
                self.gain_state.pitch()

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
