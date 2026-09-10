"""
Detection lock, arm/disable latches, and target error tracking.

Takes detections, maintains which one is locked, computes the normalised
image error and its rate, and publishes to TargetState.

No camera imports here - it works on plain Detection objects, so it can be
tested with synthetic data.
"""

import time

from config import MAIN_SIZE, MATCH_RADIUS_FRAC, RATE_ALPHA
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
    """Edge-armed, one-way latch for the Aux1 arm switch.

    Goes ARMED only on a genuine low->high transition of Aux1 that
    happens while `locked` is already True. If Aux1 is already high
    when the lock arrives, that does *not* arm - the pilot has to lower
    Aux1 and raise it again while still locked.

    Once ARMED, it stays ARMED for the rest of this run - neither Aux1
    dropping nor losing the lock disarms it. There is no software
    disarm path; only recreating this object (i.e. restarting
    main_ai.py) resets the latch.
    """

    def __init__(self):
        self._armed = False
        self._prev_high = False

    def update(self, aux1_high, locked):
        """Returns the current ARMED state."""
        edge = aux1_high and not self._prev_high
        self._prev_high = aux1_high

        if not self._armed and edge and locked:
            self._armed = True

        return self._armed


class DisableLatch:
    """One-way "kill switch" latch, for bench testing.

    Triggers permanently (for the rest of this run) the moment Aux1
    reads low while the drone is currently ARMED - lowering the arm
    switch on purpose, after having already armed, is treated as a
    deliberate abort request rather than an accidental blip (ArmLatch
    already ignores Aux1 dropping for exactly that reason).

    Once triggered, nothing clears it - not raising Aux1 again, not
    anything else. Mirrors ArmLatch's one-way design, but drives the
    opposite outcome: controller.py forces CH5 low and stops driving
    anything else once this is set. Only recreating this object (i.e.
    restarting main_ai.py) resets it.
    """

    def __init__(self):
        self._disabled = False

    def update(self, aux1_high, armed):
        """Returns the current DISABLED state."""
        if not self._disabled and armed and not aux1_high:
            self._disabled = True
        return self._disabled
