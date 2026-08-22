#!/usr/bin/env python3
"""
Unit tests for tracker.py. Runs anywhere - no Pi, no camera, no serial.
Every class in tracker.py is documented as hardware-free and testable
with synthetic data; this is that test.

    python3 test_tracker.py
"""

import sys

from tracker import AuxLock, ArmLatch, GpsRescueLatch, DisableLatch, ErrorTracker

failures = []


def check(name, cond):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}")
        failures.append(name)


class D:
    """Minimal stand-in for vision.Detection."""
    def __init__(self, box, conf):
        self.box = box
        self.conf = conf


SIZE = (640, 480)


print("\nAuxLock")
lock = AuxLock(size=SIZE)
check("starts unlocked", lock.locked is False)
check("disabled, no detections -> None", lock.update([], False) is None)
check("still unlocked after disabled update", lock.locked is False)

obj = D((10, 10, 50, 50), 0.9)
check("enabled + detection -> acquires", lock.update([obj], True) == obj.box)
check("locked after acquisition", lock.locked is True)

better = D((400, 400, 50, 50), 0.99)
check("higher-confidence newcomer elsewhere does NOT steal the lock",
     lock.update([obj, better], True) == obj.box)

moved = D((20, 10, 50, 50), 0.5)
check("tracks the same object as it moves (nearest match)",
     lock.update([moved], True) == moved.box)

check("target vanishes -> None, but stays 'locked' (searching)",
     lock.update([], True) is None)
check("still .locked while searching", lock.locked is True)

reappeared = D((22, 11, 50, 50), 0.5)
check("reappears near last known position -> re-matches",
     lock.update([reappeared], True) == reappeared.box)

check("disabling drops the lock", lock.update([obj], False) is None)
check("unlocked after disabling", lock.locked is False)

far_away = D((600, 400, 30, 30), 0.9)
check("re-enabling with an object far from history acquires fresh (highest conf)",
     lock.update([far_away, D((0, 0, 5, 5), 0.1)], True) == far_away.box)


print("\nArmLatch")
# Happy path: lock first, then raise Aux1 while locked -> arms
latch = ArmLatch()
check("not armed, not high, not locked", latch.update(False, False) is False)
check("locked, aux1 still low -> not armed", latch.update(False, True) is False)
check("aux1 raised while locked -> ARMS", latch.update(True, True) is True)
check("stays armed even if aux1 later drops", latch.update(False, False) is True)
check("stays armed even if lock is lost", latch.update(False, False) is True)
check("stays armed with aux1 high again (no fresh edge needed once armed)",
     latch.update(True, False) is True)

# Aux1 already high before the lock arrives -> must NOT arm
latch2 = ArmLatch()
check("aux1 high before lock", latch2.update(True, False) is False)
check("lock arrives while aux1 already high -> does NOT arm",
     latch2.update(True, True) is False)
check("aux1 still high, still not armed", latch2.update(True, True) is False)
latch2.update(False, True)   # lower aux1
check("raising aux1 again while locked -> NOW arms",
     latch2.update(True, True) is True)


print("\nGpsRescueLatch")
# Timeout path
gps = GpsRescueLatch(timeout=5.0)
t = 0.0
for expected_remaining in (5.0, 4.0, 3.0, 2.0, 1.0):
    triggered, remaining = gps.update(searching=True, aux4_high=False, now=t)
    check(f"t={t}: not yet triggered, remaining~{expected_remaining}",
         triggered is False and abs(remaining - expected_remaining) < 1e-9)
    t += 1.0
triggered, remaining = gps.update(searching=True, aux4_high=False, now=t)
check("hits the timeout -> triggers", triggered is True and remaining is None)
check("stays triggered afterward",
     gps.update(searching=False, aux4_high=False, now=t + 1)[0] is True)

# Returning to ARMED before the timeout resets the countdown
gps2 = GpsRescueLatch(timeout=5.0)
gps2.update(searching=True, aux4_high=False, now=0.0)
gps2.update(searching=True, aux4_high=False, now=3.0)
gps2.update(searching=False, aux4_high=False, now=4.0)   # back to ARMED
triggered, remaining = gps2.update(searching=True, aux4_high=False, now=5.0)
check("countdown restarts from the top after returning to ARMED",
     triggered is False and abs(remaining - 5.0) < 1e-9)

# Manual override: immediate, and permanent
gps3 = GpsRescueLatch(timeout=5.0)
triggered, remaining = gps3.update(searching=False, aux4_high=True, now=0.0)
check("aux4_high triggers immediately", triggered is True and remaining is None)
check("stays triggered even if aux4_high goes False",
     gps3.update(searching=False, aux4_high=False, now=1.0)[0] is True)
check("stays triggered even if the object reappears (searching=False)",
     gps3.update(searching=False, aux4_high=False, now=2.0)[0] is True)


print("\nDisableLatch")
# Aux1 low while not armed/rescue -> no effect at all (this is normal
# pre-arm state, not an abort request)
dis = DisableLatch()
check("aux1 low, not armed -> not disabled", dis.update(False, False) is False)
check("aux1 high, not armed -> still not disabled", dis.update(True, False) is False)

# Aux1 low while ARMED -> triggers
dis2 = DisableLatch()
check("aux1 high while armed -> not disabled yet", dis2.update(True, True) is False)
check("aux1 drops while armed -> DISABLED", dis2.update(False, True) is True)
check("stays disabled even if aux1 raised again",
     dis2.update(True, True) is True)
check("stays disabled even if armed_or_rescue later goes False",
     dis2.update(True, False) is True)

# Aux1 low while GPS_RESCUE (armed_or_rescue covers both) -> triggers
dis3 = DisableLatch()
check("aux1 low during GPS_RESCUE -> DISABLED",
     dis3.update(False, True) is True)   # caller passes armed or rescue


print("\nErrorTracker")
et = ErrorTracker(size=SIZE)
check("no box -> None", et.update(None, now=0.0) is None)

# Box centred exactly on frame centre -> zero error
centred_box = (320 - 25, 240 - 25, 50, 50)   # centre at (320, 240) == frame centre
ex, ey, ex_rate, ey_rate = et.update(centred_box, now=1.0)
check("centred box -> ~zero error", abs(ex) < 1e-9 and abs(ey) < 1e-9)
check("first sample -> zero rate (no prior sample)",
     ex_rate == 0.0 and ey_rate == 0.0)

# Box in the top-left quadrant -> negative ex, negative ey
tl_box = (0, 0, 20, 20)   # centre at (10, 10)
ex2, ey2, _, _ = et.update(tl_box, now=1.1)
check("top-left box -> negative ex", ex2 < 0)
check("top-left box -> negative ey", ey2 < 0)

# Losing the target resets state - reacquiring doesn't inherit a stale rate
et.update(None, now=1.2)
ex3, ey3, ex_rate3, ey_rate3 = et.update(centred_box, now=1.3)
check("re-acquiring after loss starts with zero rate (no derivative spike)",
     ex_rate3 == 0.0 and ey_rate3 == 0.0)


print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All tests passed.")
