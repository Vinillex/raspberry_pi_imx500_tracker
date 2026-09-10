"""
Detection lock, the arm latch, live PID-gain tuning, and target error
tracking.

Takes detections, maintains which one is locked, computes the normalised
image error and its rate, and publishes to TargetState. GainTuner is the
static-testing gain knob - switch/wheel positions in, a gain dict out.

No camera imports here - it works on plain Detection objects, so it can be
tested with synthetic data.
"""

import time

from config import (MAIN_SIZE, MATCH_RADIUS_FRAC, RATE_ALPHA,
                    AUX_LOW_MAX, AUX_HIGH_MIN, AUX6_DEADBAND, GAIN_LIMITS,
                    CRSF_MID, CRSF_MAX,
                    ROLL_KP, ROLL_KI, ROLL_KD, PITCH_KP, PITCH_KI, PITCH_KD)
from vision import box_center, nearest_idx


class ErrorTracker:
    """Normalised horizontal/vertical error (and smoothed rate) of a
    box's centre from the frame centre - the signal the PID controllers
    in controller.py drive roll/pitch from, via TargetState.

    No camera imports - pure box geometry, testable with synthetic data.
    """

    def __init__(self, size=MAIN_SIZE):
        self.cx = size[0] / 2.0
        self.cy = size[1] / 2.0
        self._prev_ex = 0.0
        self._prev_ey = 0.0
        self._ex_rate = 0.0
        self._ey_rate = 0.0
        self._prev_t = time.monotonic()

    def update(self, box, now=None):
        """box is (x, y, w, h) or None. Returns (ex, ey, ex_rate, ey_rate),
        or None if box is None - nothing to track, so the caller should
        hold neutral rather than coast on a stale error. Resets the
        smoothed rate whenever box is None, so a re-acquired target
        doesn't start with a derivative spike from the gap."""
        if now is None:
            now = time.monotonic()

        if box is None:
            self._prev_ex = self._prev_ey = 0.0
            self._ex_rate = self._ey_rate = 0.0
            self._prev_t = now
            return None

        dt = max(now - self._prev_t, 1e-3)
        self._prev_t = now

        cx, cy = box_center(box)
        ex = (cx - self.cx) / self.cx
        ey = (cy - self.cy) / self.cy

        raw_ex_rate = (ex - self._prev_ex) / dt
        raw_ey_rate = (ey - self._prev_ey) / dt
        self._ex_rate = (1 - RATE_ALPHA) * self._ex_rate + RATE_ALPHA * raw_ex_rate
        self._ey_rate = (1 - RATE_ALPHA) * self._ey_rate + RATE_ALPHA * raw_ey_rate
        self._prev_ex, self._prev_ey = ex, ey

        return ex, ey, self._ex_rate, self._ey_rate


class AuxLock:
    """Confidence-based auto-lock, driven by a switch instead of a keyboard.

    While enabled, locks onto the highest-confidence detection the first
    time one is available, then keeps following that same object
    frame-to-frame by nearest-centre match - a newly-appeared
    higher-confidence detection never steals the lock. Disabling drops
    the lock immediately; re-enabling acquires fresh.

    No camera imports here - works on plain Detection objects, so it can
    be tested with synthetic data.
    """

    def __init__(self, size=MAIN_SIZE):
        w, _h = size
        self.match_r2 = (MATCH_RADIUS_FRAC * w) ** 2
        self._center = None

    def update(self, detections, enabled):
        """Returns the locked box, or None if nothing is currently
        matched (disabled, nothing to lock onto yet, or momentarily
        lost - use `.locked` to tell those two apart)."""
        if not enabled:
            self._center = None
            return None

        if self._center is None:
            if not detections:
                return None
            best = max(detections, key=lambda d: d.conf)
            self._center = box_center(best.box)
            return best.box

        i, d2 = nearest_idx(detections, self._center)
        if i < 0 or d2 is None or d2 > self.match_r2:
            return None   # keep self._center, wait to reacquire

        box = detections[i].box
        self._center = box_center(box)
        return box

    @property
    def locked(self):
        """True once a lock has been acquired (even if momentarily
        unmatched this frame); False if disabled or never acquired."""
        return self._center is not None


class ArmLatch:
    """Edge-armed latch for the Aux1 arm switch.

    On this static-testing branch it is deliberately NOT one-way, so the
    airframe can be armed and disarmed repeatedly between tuning passes:

      * Arms on a low->high edge of Aux1 that lands while `locked` is
        already True. Aux1 already high when the lock arrives does not
        arm - lower it and raise it again while locked.
      * Lowering Aux1 disarms immediately (ready for the next variable).
      * Losing the lock while Aux1 stays high does NOT disarm - that is
        the SEARCHING state; only the switch dropping disarms.
    """

    def __init__(self):
        self._armed = False
        self._prev_high = False

    def update(self, aux1_high, locked):
        """Returns the current ARMED state."""
        edge = aux1_high and not self._prev_high
        self._prev_high = aux1_high

        if not aux1_high:
            self._armed = False
        elif edge and locked:
            self._armed = True

        return self._armed


class GainTuner:
    """Live PID-gain tuning for static bench testing (this branch only).

    Selection: Aux4 chooses the axis (low -> roll, high -> pitch); Aux2
    (roll) or Aux3 (pitch) is a 3-position switch choosing Kp / Ki / Kd.
    Adjustment: Aux6 is a spring-return scroll wheel - held forward it
    ramps the selected gain up, held back ramps it down, centred it
    holds. Per-gain rates and limits come from config.GAIN_LIMITS.

    Tuned values persist across arm/disarm cycles; only recreating this
    object (restarting the script) resets them to the config defaults.

    No hardware here - raw CRSF ints in, a gain dict out - so it's
    unit-tested with synthetic values.
    """

    _GAIN_BY_POS = ("kp", "ki", "kd")

    def __init__(self):
        self._g = {
            "roll_kp": float(ROLL_KP), "roll_ki": float(ROLL_KI),
            "roll_kd": float(ROLL_KD), "pitch_kp": float(PITCH_KP),
            "pitch_ki": float(PITCH_KI), "pitch_kd": float(PITCH_KD),
        }

    @staticmethod
    def _switch3(value):
        """3-position switch value -> 0 (low) / 1 (mid) / 2 (high)."""
        if value <= AUX_LOW_MAX:
            return 0
        if value >= AUX_HIGH_MIN:
            return 2
        return 1

    @staticmethod
    def deflection(aux6):
        """Aux6 as a signed fraction of full throw in [-1.0, +1.0]:
        0 centred, +1 fully forward (CRSF_MAX), -1 fully back."""
        d = (aux6 - CRSF_MID) / float(CRSF_MAX - CRSF_MID)
        return max(-1.0, min(1.0, d))

    def selected(self, aux2, aux3, aux4):
        """Name of the gain the switches currently point at, e.g.
        'roll_kd'."""
        axis = "pitch" if aux4 >= AUX_HIGH_MIN else "roll"
        knob = aux3 if axis == "pitch" else aux2
        return f"{axis}_{self._GAIN_BY_POS[self._switch3(knob)]}"

    def update(self, aux2, aux3, aux4, aux6, dt):
        """Ramp the selected gain by the wheel position over `dt` seconds.
        Returns (gains_dict, selected_name, centred)."""
        name = self.selected(aux2, aux3, aux4)
        d = self.deflection(aux6)
        centred = abs(d) <= AUX6_DEADBAND
        if not centred:
            lo, hi, rate = GAIN_LIMITS[name]
            stepped = self._g[name] + d * rate * dt
            self._g[name] = max(lo, min(hi, stepped))
        return dict(self._g), name, centred

    @property
    def gains(self):
        """Plain dict copy of the six current gains."""
        return dict(self._g)
