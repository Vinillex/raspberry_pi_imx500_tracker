#!/usr/bin/env python3
"""
Unit tests for tracker.py. Runs anywhere - no Pi, no camera, no serial.
Every class in tracker.py is documented as hardware-free and testable
with synthetic data; this is that test.

    python3 test_tracker.py
"""

import sys

from tracker import AuxLock, ArmLatch, GainTuner, ErrorTracker
from config import CRSF_MIN, CRSF_MID, CRSF_MAX, GAIN_LIMITS, ROLL_KP

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
LOW, MID, HIGH = CRSF_MIN, CRSF_MID, CRSF_MAX


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


print("\nArmLatch (static-testing: NOT one-way - lowering Aux1 disarms)")
latch = ArmLatch()
check("not armed, not high, not locked", latch.update(False, False) is False)
check("locked, aux1 still low -> not armed", latch.update(False, True) is False)
check("aux1 raised while locked -> ARMS", latch.update(True, True) is True)
check("stays armed while aux1 held high and the lock is lost (SEARCHING)",
     latch.update(True, False) is True)
check("aux1 dropped -> DISARMS", latch.update(False, False) is False)
check("aux1 high again but not locked -> stays disarmed",
     latch.update(True, False) is False)
check("aux1 still high, now locked, but no fresh low->high edge -> stays disarmed",
     latch.update(True, True) is False)
latch.update(False, True)   # lower aux1
check("lower then raise while locked -> RE-ARMS", latch.update(True, True) is True)
check("lower aux1 again -> disarms again", latch.update(False, True) is False)

# Aux1 already high before the lock arrives -> must NOT arm
latch2 = ArmLatch()
check("aux1 high before lock -> not armed", latch2.update(True, False) is False)
check("lock arrives while aux1 already high -> does NOT arm",
     latch2.update(True, True) is False)
latch2.update(False, True)   # lower aux1
check("raising aux1 again while locked -> NOW arms",
     latch2.update(True, True) is True)


print("\nGainTuner - selection matrix (Aux4 axis, Aux2/Aux3 gain)")
gt = GainTuner()
check("Aux2 low,  Aux4 low  -> roll_kp",  gt.selected(LOW,  MID, LOW)  == "roll_kp")
check("Aux2 mid,  Aux4 low  -> roll_ki",  gt.selected(MID,  MID, LOW)  == "roll_ki")
check("Aux2 high, Aux4 low  -> roll_kd",  gt.selected(HIGH, MID, LOW)  == "roll_kd")
check("Aux3 low,  Aux4 high -> pitch_kp", gt.selected(MID, LOW,  HIGH) == "pitch_kp")
check("Aux3 mid,  Aux4 high -> pitch_ki", gt.selected(MID, MID,  HIGH) == "pitch_ki")
check("Aux3 high, Aux4 high -> pitch_kd", gt.selected(MID, HIGH, HIGH) == "pitch_kd")
check("Aux2 is ignored while Aux4 selects pitch",
     gt.selected(HIGH, LOW, HIGH) == "pitch_kp")
check("Aux3 is ignored while Aux4 selects roll",
     gt.selected(LOW, HIGH, LOW) == "roll_kp")


print("\nGainTuner - Aux6 deflection")
check("centre -> 0.0", abs(GainTuner.deflection(CRSF_MID)) < 1e-9)
check("full forward -> +1.0", abs(GainTuner.deflection(CRSF_MAX) - 1.0) < 1e-9)
check("full back -> -1.0 (clamped)", GainTuner.deflection(CRSF_MIN) == -1.0)
check("beyond range clamps to +1.0", GainTuner.deflection(9999) == 1.0)


print("\nGainTuner - ramping the selected gain")
gt = GainTuner()
lo, hi, rate = GAIN_LIMITS["roll_kp"]
start = gt.gains["roll_kp"]
g, name, centred = gt.update(LOW, MID, LOW, CRSF_MAX, 1.0)   # full forward, 1 s
check("reports the selected gain name", name == "roll_kp")
check("full throw -> not centred", centred is False)
check("full-forward for 1 s ramps roll_kp up by ~rate",
     abs(g["roll_kp"] - (start + rate)) < 1e-6)
check("untouched gains keep their defaults",
     g["pitch_kd"] == GainTuner().gains["pitch_kd"])
g, _, _ = gt.update(LOW, MID, LOW, CRSF_MIN, 1.0)           # full back, 1 s
check("full-back for 1 s ramps roll_kp back down by ~rate",
     abs(g["roll_kp"] - start) < 1e-6)
g, _, _ = gt.update(LOW, MID, LOW, CRSF_MAX, 0.5)           # half the time
check("ramp scales with dt", abs(g["roll_kp"] - (start + 0.5 * rate)) < 1e-6)


print("\nGainTuner - allow_ramp=False (DETECTING): select only, never adjust")
gt = GainTuner()
held = gt.gains["roll_kd"]
for _ in range(50):
    g, name, centred = gt.update(HIGH, MID, LOW, CRSF_MAX, 1.0, allow_ramp=False)
check("selection still tracked while ramping is disabled", name == "roll_kd")
check("wheel deflection still reported", centred is False)
check("gain value is left completely untouched", g["roll_kd"] == held)
# and it starts ramping again the moment ramping is re-enabled (ARMED)
g, _, _ = gt.update(HIGH, MID, LOW, CRSF_MAX, 1.0, allow_ramp=True)
_, _, r = GAIN_LIMITS["roll_kd"]
check("re-enabling ramp resumes adjustment", g["roll_kd"] == held + r)


print("\nGainTuner - centre deadband holds the value")
gt = GainTuner()
before = gt.gains["roll_ki"]
g, name, centred = gt.update(MID, MID, LOW, CRSF_MID, 1.0)
check("centred wheel -> centred True", centred is True)
check("centred wheel -> gain unchanged", g["roll_ki"] == before)
check("selection still tracked while centred", name == "roll_ki")
near = CRSF_MID + int(0.03 * (CRSF_MAX - CRSF_MID))   # inside AUX6_DEADBAND
g, _, centred = gt.update(MID, MID, LOW, near, 1.0)
check("small off-centre within the deadband still holds",
     centred is True and g["roll_ki"] == before)


print("\nGainTuner - clamps to the per-gain envelope")
gt = GainTuner()
lo, hi, _ = GAIN_LIMITS["pitch_kd"]
for _ in range(500):
    g, _, _ = gt.update(MID, HIGH, HIGH, CRSF_MAX, 1.0)
check("hammering the wheel up clamps at the gain's max", g["pitch_kd"] == hi)
for _ in range(500):
    g, _, _ = gt.update(MID, HIGH, HIGH, CRSF_MIN, 1.0)
check("hammering it down clamps at the gain's min", g["pitch_kd"] == lo)


print("\nGainTuner - values persist across calls and selection changes")
gt = GainTuner()
gt.update(LOW, MID, LOW, CRSF_MAX, 0.5)          # bump roll_kp
bumped = gt.gains["roll_kp"]
check("roll_kp stays bumped above its default", bumped > ROLL_KP)
gt.update(MID, HIGH, HIGH, CRSF_MID, 1.0)        # switch to pitch_kd, wheel centred
check("changing the selection doesn't disturb roll_kp",
     gt.gains["roll_kp"] == bumped)


print("\nErrorTracker")
et = ErrorTracker(size=SIZE)
check("no box -> None", et.update(None, now=0.0) is None)

centred_box = (320 - 25, 240 - 25, 50, 50)   # centre at (320, 240) == frame centre
ex, ey, ex_rate, ey_rate = et.update(centred_box, now=1.0)
check("centred box -> ~zero error", abs(ex) < 1e-9 and abs(ey) < 1e-9)
check("first sample -> zero rate (no prior sample)",
     ex_rate == 0.0 and ey_rate == 0.0)

tl_box = (0, 0, 20, 20)   # centre at (10, 10)
ex2, ey2, _, _ = et.update(tl_box, now=1.1)
check("top-left box -> negative ex", ex2 < 0)
check("top-left box -> negative ey", ey2 < 0)

et.update(None, now=1.2)
ex3, ey3, ex_rate3, ey_rate3 = et.update(centred_box, now=1.3)
check("re-acquiring after loss starts with zero rate (no derivative spike)",
     ex_rate3 == 0.0 and ey_rate3 == 0.0)


print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All tests passed.")
