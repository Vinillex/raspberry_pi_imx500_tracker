#!/usr/bin/env python3
"""
Unit tests for controller.py. Runs anywhere - no Pi, no camera, no serial.
controller.py's own module docstring says it's testable without hardware;
this is that test. ArmLatch/TrackController together decide whether the
aircraft is under manual or fully-autonomous control, so this is the
single most safety-critical logic in the project.

    python3 test_controller.py
"""

import sys

from controller import PID, TrackController
from state import TargetState, ArmState, RescueState, DisableState
from config import (CH_ROLL, CH_PITCH, CH_THROTTLE, CH_AUX1, CH_AUX4,
                    CH_AUX5, CH_AUX6, CRSF_MIN, CRSF_MID, CRSF_MAX)

failures = []


def check(name, cond):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}")
        failures.append(name)


def make_channels():
    """16 channels with distinctive, non-CRSF_MID pilot stick values, so
    a passthrough vs. an override are trivially distinguishable."""
    ch = [CRSF_MID] * 16
    ch[CH_ROLL] = 1700
    ch[CH_PITCH] = 300
    ch[CH_THROTTLE] = 500
    return ch


print("\nPID")
pid = PID(kp=10.0, ki=5.0, kd=2.0, i_max=3.0)
out = pid.update(error=0.5, rate=0.2, dt=0.1)
check("P+I+D combine correctly on the first sample",
     abs(out - (10.0 * 0.5 + 5.0 * (0.5 * 0.1) + 2.0 * 0.2)) < 1e-9)

pid2 = PID(kp=10.0, ki=0.0, kd=2.0, i_max=100.0)
out2 = pid2.update(error=0.5, rate=0.2, dt=0.1)
check("ki=0 -> pure P+D", abs(out2 - (10.0 * 0.5 + 2.0 * 0.2)) < 1e-9)

pid3 = PID(kp=0.0, ki=5.0, kd=0.0, i_max=3.0)
for _ in range(50):
    pid3.update(error=1.0, rate=0.0, dt=0.1)   # raw accumulation would be 25.0
check("integral clamps at i_max, not the raw accumulated value",
     pid3._integral == 3.0)

pid3.reset()
check("reset() zeroes the integral", pid3._integral == 0.0)


def new_controller(with_disable=False):
    """with_disable=False matches the constructor's own default
    (disable_state=None) - the "kill switch not wired up" case, which
    must mean DISABLED can never trigger. with_disable=True additionally
    returns the DisableState so a test can simulate main_ai.py's
    tracker.DisableLatch firing."""
    target = TargetState()
    arm = ArmState()
    rescue = RescueState()
    disable = DisableState() if with_disable else None
    c = TrackController(target, arm, rescue, disable)
    return (c, target, arm, rescue, disable) if with_disable else (c, target, arm, rescue)


print("\nTrackController - not armed")
c, target, arm, rescue = new_controller()
out = c.apply(make_channels())
check("pilot has full manual control on roll/pitch/throttle",
     out[CH_ROLL] == 1700 and out[CH_PITCH] == 300 and out[CH_THROTTLE] == 500)
check("Aux5 neutralised even when not armed", out[CH_AUX5] == CRSF_MID)
check("Aux6 neutralised even when not armed", out[CH_AUX6] == CRSF_MID)
check("Aux1 forced low (not a raw passthrough) when not armed and not locked",
     out[CH_AUX1] == CRSF_MIN)
check("Aux4 forced low when not in rescue", out[CH_AUX4] == CRSF_MIN)

# Raw Aux1/Aux4 values on the input side must have NO bearing on the
# output - controller.py never reads them, only arm_state/rescue_state.
ch_high_aux = make_channels()
ch_high_aux[CH_AUX1] = CRSF_MAX
ch_high_aux[CH_AUX4] = CRSF_MAX
out = c.apply(ch_high_aux)
check("raw Aux1=high input is ignored while not armed", out[CH_AUX1] == CRSF_MIN)
check("raw Aux4=high input is ignored while not in rescue", out[CH_AUX4] == CRSF_MIN)


print("\nTrackController - CH5 mirrors LOCKED live, before ARMED")
c, target, arm, rescue = new_controller()
target.publish(ex=0.1, ey=0.1, ex_rate=0.0, ey_rate=0.0)   # LOCKED, not armed
out = c.apply(make_channels())
check("CH5 goes high on LOCKED alone, with no arm switch involved at all",
     out[CH_AUX1] == CRSF_MAX)
check("pilot still has full manual roll/pitch/throttle - LOCKED alone "
     "doesn't engage tracking, only CH5 changes",
     out[CH_ROLL] == 1700 and out[CH_PITCH] == 300 and out[CH_THROTTLE] == 500)

target.invalidate()   # lock lost - unlike ARMED, this is NOT one-way
out = c.apply(make_channels())
check("CH5 drops back low the instant LOCKED is lost, pre-arm",
     out[CH_AUX1] == CRSF_MIN)

target.publish(ex=0.1, ey=0.1, ex_rate=0.0, ey_rate=0.0)   # re-lock
out = c.apply(make_channels())
check("CH5 goes back high on re-lock - freely retestable, unlike ARMED",
     out[CH_AUX1] == CRSF_MAX)

target.invalidate()
arm.set(True)   # now actually armed, with no lock at the moment it fires
out = c.apply(make_channels())
check("CH5 stays high once ARMED, even though LOCKED is false right now",
     out[CH_AUX1] == CRSF_MAX)
out = c.apply(make_channels())
check("CH5 stays high on a later call with LOCKED still false - ARMED "
     "is sticky regardless of LOCKED afterward",
     out[CH_AUX1] == CRSF_MAX)


print("\nTrackController - ARMED, SEARCHING (no target)")
c, target, arm, rescue = new_controller()
arm.set(True)
out = c.apply(make_channels())
check("Aux1 forced high once armed", out[CH_AUX1] == CRSF_MAX)
check("roll neutral while searching", out[CH_ROLL] == CRSF_MID)
check("pitch neutral while searching", out[CH_PITCH] == CRSF_MID)
check("throttle is left as a raw pilot passthrough while searching "
     "(no throttle automation - arming is being isolated for testing)",
     out[CH_THROTTLE] == 500)
check("pilot's roll/pitch stick values are ignored, but throttle isn't",
     out[CH_ROLL] != 1700 and out[CH_PITCH] != 300)


print("\nTrackController - ARMED, actively tracking")
c, target, arm, rescue = new_controller()
arm.set(True)
target.publish(ex=0.5, ey=-0.3, ex_rate=0.0, ey_rate=0.0)
out = c.apply(make_channels())
check("throttle stays a raw pilot passthrough while actively tracking too",
     out[CH_THROTTLE] == 500)
check("positive horizontal error pushes roll away from centre",
     out[CH_ROLL] != CRSF_MID)
check("negative vertical error pushes pitch away from centre",
     out[CH_PITCH] != CRSF_MID)
check("roll/pitch are a full override, not the pilot's stick values",
     out[CH_ROLL] != 1700 and out[CH_PITCH] != 300)


print("\nTrackController - GPS_RESCUE (even with a target still visible)")
c, target, arm, rescue = new_controller()
arm.set(True)
target.publish(ex=0.5, ey=-0.3, ex_rate=0.0, ey_rate=0.0)
rescue.set(True)
out = c.apply(make_channels())
check("Aux4 forced high once rescue latches", out[CH_AUX4] == CRSF_MAX)
check("roll goes neutral despite a visible/locked target",
     out[CH_ROLL] == CRSF_MID)
check("pitch goes neutral despite a visible/locked target",
     out[CH_PITCH] == CRSF_MID)
check("throttle is left as a raw pilot passthrough during GPS_RESCUE too",
     out[CH_THROTTLE] == 500)


print("\nTrackController - DISABLED (bench-test kill switch)")
c, target, arm, rescue, disable = new_controller(with_disable=True)
arm.set(True)
rescue.set(True)
target.publish(ex=0.5, ey=-0.3, ex_rate=0.0, ey_rate=0.0)   # strong error
disable.set(True)   # simulates tracker.DisableLatch firing in main_ai.py
out = c.apply(make_channels())
check("CH5 forced low once DISABLED, despite ARMED being true",
     out[CH_AUX1] == CRSF_MIN)
check("CH8 forced low once DISABLED, despite rescue being true",
     out[CH_AUX4] == CRSF_MIN)
check("Aux5/Aux6 still neutralised while DISABLED",
     out[CH_AUX5] == CRSF_MID and out[CH_AUX6] == CRSF_MID)
check("roll/pitch/throttle are full pilot passthrough once DISABLED, "
     "even with a strong locked-target error and ARMED/rescue both true",
     out[CH_ROLL] == 1700 and out[CH_PITCH] == 300 and out[CH_THROTTLE] == 500)

# Nothing reverses it - not lowering arm/rescue, not a fresh target error.
arm.set(False)
rescue.set(False)
out = c.apply(make_channels())
check("DISABLED persists even if arm_state/rescue_state later go False - "
     "the kill switch does not depend on them once latched",
     out[CH_AUX1] == CRSF_MIN and out[CH_AUX4] == CRSF_MIN)

print("\nTrackController - disable_state=None means DISABLED can never fire")
c, target, arm, rescue = new_controller()   # disable_state=None, the default
arm.set(True)
target.publish(ex=0.5, ey=-0.3, ex_rate=0.0, ey_rate=0.0)
out = c.apply(make_channels())
check("armed+tracking behaves exactly as if the kill switch didn't exist",
     out[CH_AUX1] == CRSF_MAX and out[CH_ROLL] != CRSF_MID)


print("\nTrackController - stale target counts as no target")
from config import VISION_TIMEOUT
c, target, arm, rescue = new_controller()
arm.set(True)
target.publish(ex=0.5, ey=-0.3, ex_rate=0.0, ey_rate=0.0)
# Force the published stamp to look old by publishing then waiting is
# slow for a test; instead poke the private timestamp directly, since
# this module explicitly documents itself as internal-state-testable.
target._stamp -= (VISION_TIMEOUT + 1.0)
out = c.apply(make_channels())
check("stale target -> treated as SEARCHING, not tracked",
     out[CH_ROLL] == CRSF_MID and out[CH_THROTTLE] == 500)


print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All tests passed.")
