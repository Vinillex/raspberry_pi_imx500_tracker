#!/usr/bin/env python3
"""
Unit tests for tracker.py. Runs anywhere - no Pi, no camera, no serial.
Every class in tracker.py is documented as hardware-free and testable
with synthetic data; this is that test.

    python3 test_tracker.py
"""

import sys

from tracker import AuxLock, ArmLatch, GainTuner, ErrorTracker
from config import CRSF_MIN, CRSF_MID, CRSF_MAX, GAIN_SWEEP, ROLL_KP

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


HALF_SWEEP_FRAC = (CRSF_MAX - CRSF_MID) / (CRSF_MAX - CRSF_MIN)   # centre -> full


print("\nGainTuner - Aux6 is a relative control (only movement counts)")
gt = GainTuner()
span = GAIN_SWEEP["roll_kp"]
start = gt.gains["roll_kp"]
g, name, _ = gt.update(LOW, MID, LOW, CRSF_MID)      # first call only anchors
check("reports the selection", name == "roll_kp")
check("first call moves nothing (no prior wheel position)",
     g["roll_kp"] == start)
g, _, _ = gt.update(LOW, MID, LOW, CRSF_MAX)         # centre -> full forward
step = HALF_SWEEP_FRAC * span
check("turning the wheel forward raises the selected gain",
     abs(g["roll_kp"] - (start + step)) < 1e-6)
g, _, _ = gt.update(LOW, MID, LOW, CRSF_MAX)         # hold it there
check("holding the wheel still holds the value (not a rate control)",
     abs(g["roll_kp"] - (start + step)) < 1e-6)
g, _, _ = gt.update(LOW, MID, LOW, CRSF_MID)         # turn back to centre
check("turning back the same distance undoes the change",
     abs(g["roll_kp"] - start) < 1e-6)
check("untouched gains never moved",
     g["roll_kd"] == GainTuner().gains["roll_kd"])


print("\nGainTuner - allow_ramp=False (DETECTING): select only, wheel inert")
gt = GainTuner()
held = gt.gains["roll_kd"]
gt.update(HIGH, MID, LOW, CRSF_MID, allow_ramp=False)
for pos in (1200, 1500, CRSF_MAX, 1400, CRSF_MID):        # wheel wandering
    g, name, _ = gt.update(HIGH, MID, LOW, pos, allow_ramp=False)
check("selection still tracked while disarmed", name == "roll_kd")
check("wheel movement while disarmed never touches the value",
     g["roll_kd"] == held)


print("\nGainTuner - disarm commits, re-centre, re-arm continues from there")
gt = GainTuner()
span = GAIN_SWEEP["roll_kd"]
v0 = gt.gains["roll_kd"]
step = HALF_SWEEP_FRAC * span
gt.update(HIGH, MID, LOW, CRSF_MID, allow_ramp=True)          # arm, anchor centre
g, _, _ = gt.update(HIGH, MID, LOW, CRSF_MAX, allow_ramp=True)   # wheel -> forward
tuned = g["roll_kd"]
check("wheel forward raised the gain", abs(tuned - (v0 + step)) < 1e-6)
g, _, _ = gt.update(HIGH, MID, LOW, CRSF_MAX, allow_ramp=False)  # DISARM
g, _, _ = gt.update(HIGH, MID, LOW, CRSF_MID, allow_ramp=False)  # re-centre wheel
check("disarm + re-centring the wheel leave the tuned value put",
     g["roll_kd"] == tuned)
g, _, _ = gt.update(HIGH, MID, LOW, CRSF_MID, allow_ramp=True)   # RE-ARM, centred
check("re-arming doesn't jump the value", g["roll_kd"] == tuned)
g, _, _ = gt.update(HIGH, MID, LOW, CRSF_MAX, allow_ramp=True)   # wheel forward
check("further turns continue from the tuned value, not the default",
     abs(g["roll_kd"] - (tuned + step)) < 1e-6)


print("\nGainTuner - centre deadband (the Aux5 lock interlock)")
gt = GainTuner()
_, _, centred = gt.update(MID, MID, LOW, CRSF_MID, allow_ramp=True)
check("wheel at centre -> centred True", centred is True)
near = CRSF_MID + int(0.03 * (CRSF_MAX - CRSF_MID))   # inside AUX6_DEADBAND
_, _, centred = gt.update(MID, MID, LOW, near, allow_ramp=True)
check("small offset within the deadband -> still centred", centred is True)
_, _, centred = gt.update(MID, MID, LOW, CRSF_MAX, allow_ramp=True)
check("wheel turned well off centre -> not centred", centred is False)


print("\nGainTuner - ratcheting up has NO cap; ratcheting down floors at 0")
gt = GainTuner()
for _ in range(10):
    gt.update(MID, HIGH, HIGH, CRSF_MID, allow_ramp=False)   # re-centre (absorbed)
    gt.update(MID, HIGH, HIGH, CRSF_MID, allow_ramp=True)    # arm, anchor centre
    g, _, _ = gt.update(MID, HIGH, HIGH, CRSF_MAX, allow_ramp=True)   # turn up
check("ratcheting up keeps climbing past any old cap (was 400)",
     g["pitch_kd"] > 1000.0)
gt = GainTuner()
for _ in range(10):
    gt.update(MID, HIGH, HIGH, CRSF_MID, allow_ramp=False)
    gt.update(MID, HIGH, HIGH, CRSF_MID, allow_ramp=True)
    g, _, _ = gt.update(MID, HIGH, HIGH, CRSF_MIN, allow_ramp=True)   # turn down
check("ratcheting down can't go below 0", g["pitch_kd"] == 0.0)


print("\nGainTuner - a selection change doesn't leak wheel movement")
gt = GainTuner()
gt.update(LOW, MID, LOW, CRSF_MID, allow_ramp=True)   # roll_kp selected + anchored
kd0 = gt.gains["roll_kd"]
g, name, _ = gt.update(HIGH, MID, LOW, CRSF_MAX, allow_ramp=True)   # flip + turn
check("the frame the switch flips doesn't move the newly-selected gain",
     name == "roll_kd" and g["roll_kd"] == kd0)
gt.update(HIGH, MID, LOW, CRSF_MID, allow_ramp=True)   # re-anchor at centre
g, _, _ = gt.update(HIGH, MID, LOW, CRSF_MAX, allow_ramp=True)   # now turn up
check("wheel works normally once the selection has settled",
     g["roll_kd"] > kd0)


print("\nGainTuner - tuned values persist for the rest of the run")
gt = GainTuner()
gt.update(LOW, MID, LOW, CRSF_MID, allow_ramp=True)
gt.update(LOW, MID, LOW, 1300, allow_ramp=True)       # nudge roll_kp up
bumped = gt.gains["roll_kp"]
check("roll_kp moved off its default", bumped != ROLL_KP)
gt.update(MID, HIGH, HIGH, CRSF_MID, allow_ramp=True)  # select pitch_kd instead
check("selecting another gain leaves roll_kp where it was",
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
