"""
Target selection and lock state machine.

Takes detections, maintains which one is selected/locked, computes the
normalised image error and its rate, and publishes to TargetState.

No camera imports here - it works on plain Detection objects, so it can be
tested with synthetic data.
"""

import time

from config import MAIN_SIZE, MATCH_RADIUS_FRAC, RATE_ALPHA, GPS_RESCUE_TIMEOUT
from vision import box_center, nearest_idx

MODE_SELECT = 0
MODE_TRACK = 1


class Tracker:
    def __init__(self, target_state, size=MAIN_SIZE):
        self.target = target_state
        self.w, self.h = size
        self.cx = self.w / 2.0
        self.cy = self.h / 2.0

        match_radius = MATCH_RADIUS_FRAC * self.w
        self.match_r2 = match_radius ** 2

        self.mode = MODE_SELECT
        self.sel_center = None      # highlighted candidate (SELECT)
        self.locked_center = None   # tracked target (TRACK)
        self.sel_idx = -1

        self._prev_ex = 0.0
        self._prev_ey = 0.0
        self._ex_rate = 0.0
        self._ey_rate = 0.0
        self._prev_t = time.monotonic()

        # Last computed values, for display
        self.ex = 0.0
        self.ey = 0.0
        self.target_box = None
        self.searching = False
        self.last_ordered = []

    # ----------------------------------------------------------------
    def update(self, detections):
        """
        Feed a fresh list of detections. Returns the ordered list
        (left to right) so the caller can draw them.
        """
        now = time.monotonic()
        dt = max(now - self._prev_t, 1e-3)
        self._prev_t = now

        ordered = sorted(detections, key=lambda d: box_center(d.box)[0])
        self.last_ordered = ordered
        self.sel_idx = -1
        self.target_box = None
        self.searching = False

        if self.mode == MODE_SELECT:
            self.target.invalidate()
            if ordered and self.sel_center is not None:
                i, d2 = nearest_idx(ordered, self.sel_center)
                if d2 is not None and d2 <= self.match_r2:
                    self.sel_idx = i
                    self.sel_center = box_center(ordered[i].box)
            return ordered

        # ---- MODE_TRACK ----
        tgt = None
        if ordered and self.locked_center is not None:
            i, d2 = nearest_idx(ordered, self.locked_center)
            if d2 is not None and d2 <= self.match_r2:
                tgt = ordered[i]

        if tgt is None:
            # Target lost. Stop commanding immediately rather than
            # coasting on the last known error.
            self.searching = True
            self.target.invalidate()
            self._ex_rate = self._ey_rate = 0.0
            return ordered

        self.locked_center = box_center(tgt.box)
        self.target_box = tgt.box
        tcx, tcy = self.locked_center

        self.ex = (tcx - self.cx) / self.cx
        self.ey = (tcy - self.cy) / self.cy

        raw_ex_rate = (self.ex - self._prev_ex) / dt
        raw_ey_rate = (self.ey - self._prev_ey) / dt
        self._ex_rate = ((1 - RATE_ALPHA) * self._ex_rate
                         + RATE_ALPHA * raw_ex_rate)
        self._ey_rate = ((1 - RATE_ALPHA) * self._ey_rate
                         + RATE_ALPHA * raw_ey_rate)
        self._prev_ex, self._prev_ey = self.ex, self.ey

        self.target.publish(self.ex, self.ey, self._ex_rate, self._ey_rate)
        return ordered

    # ----------------------------------------------------------------
    def cycle_selection(self, ordered):
        """Advance the highlighted candidate (SELECT mode only)."""
        if self.mode != MODE_SELECT or not ordered:
            return
        self.sel_idx = 0 if self.sel_idx < 0 else (self.sel_idx + 1) % len(ordered)
        self.sel_center = box_center(ordered[self.sel_idx].box)

    def toggle_lock(self, ordered):
        """SELECT -> TRACK on a valid candidate, or TRACK -> SELECT."""
        if self.mode == MODE_SELECT:
            if 0 <= self.sel_idx < len(ordered):
                self.locked_center = box_center(ordered[self.sel_idx].box)
                self._prev_ex = self._prev_ey = 0.0
                self._ex_rate = self._ey_rate = 0.0
                self.mode = MODE_TRACK
        else:
            self.release()

    def release(self):
        self.mode = MODE_SELECT
        self.sel_center = None
        self.locked_center = None
        self.target_box = None
        self.target.invalidate()


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


class GpsRescueLatch:
    """One-way latch for the GPS-rescue failsafe.

    Triggers permanently (for the rest of this run) on either:
      - continuous SEARCHING (ARMED with no locked box) for
        `timeout` seconds without returning to ARMED, or
      - Aux4 going high - a manual override with no interlock, unlike
        the arm switch.

    Once triggered, nothing clears it - not Aux4 going low, not the
    object reappearing. Mirrors ArmLatch's one-way design.
    """

    def __init__(self, timeout=GPS_RESCUE_TIMEOUT):
        self.timeout = timeout
        self._triggered = False
        self._searching_since = None

    def update(self, searching, aux4_high, now=None):
        """searching - True while ARMED with no locked box this frame.
        aux4_high  - raw Aux4 switch state (manual override trigger).
        now        - monotonic timestamp; defaults to time.monotonic().

        Returns (triggered, remaining). `remaining` is the seconds left
        on the searching countdown while it's running, else None (not
        searching, or already triggered)."""
        if now is None:
            now = time.monotonic()

        if aux4_high:
            self._triggered = True

        remaining = None
        if not self._triggered:
            if searching:
                if self._searching_since is None:
                    self._searching_since = now
                elapsed = now - self._searching_since
                if elapsed >= self.timeout:
                    self._triggered = True
                else:
                    remaining = self.timeout - elapsed
            else:
                self._searching_since = None

        return self._triggered, remaining
