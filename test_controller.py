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
from state import TargetState, ArmState, GainState
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
    ch[CH_AUX4] = 777    # gain-tuner selector - controller passes it through
    ch[CH_AUX6] = 1234   # gain-tuner wheel - controller passes it through
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

# Ki=0 must NOT let the accumulator wind up (it would kick when Ki is
# later dialled up live from the tuning wheel).
pid4 = PID(kp=1.0, ki=0.0, kd=0.0, i_max=100.0)
for _ in range(20):
    pid4.update(error=1.0, rate=0.0, dt=0.1)
check("Ki=0 -> integral stays parked at 0", pid4._integral == 0.0)
pid4.ki = 5.0   # simulate the wheel raising Ki
out4 = pid4.update(error=1.0, rate=0.0, dt=0.1)
check("raising Ki live -> integral starts fresh, no accumulated kick",
     abs(out4 - (1.0 * 1.0 + 5.0 * (1.0 * 0.1))) < 1e-9)


def new_controller(with_gains=False):
    """with_gains=False matches the constructor's own default
    (gain_state=None) - the PIDs keep the config-default gains baked in
    at construction. with_gains=True additionally returns a GainState so
    a test can push live gains the way main_ai.py's tracker.GainTuner
    does."""
    target = TargetState()
    arm = ArmState()
    gain = GainState() if with_gains else None
    c = TrackController(target, arm, gain)
    return (c, target, arm, gain) if with_gains else (c, target, arm)


print("\nTrackController - not armed")
c, target, arm = new_controller()
out = c.apply(make_channels())
check("pilot has full manual control on roll/pitch/throttle",
     out[CH_ROLL] == 1700 and out[CH_PITCH] == 300 and out[CH_THROTTLE] == 500)
check("Aux5 neutralised even when not armed", out[CH_AUX5] == CRSF_MID)
check("Aux4 (tuner selector) passes straight through to the FC",
     out[CH_AUX4] == 777)
check("Aux6 (tuner wheel) passes straight through to the FC",
     out[CH_AUX6] == 1234)
check("Aux1 forced low (not a raw passthrough) when not armed",
     out[CH_AUX1] == CRSF_MIN)

# The raw Aux1 value on the input side must have NO bearing on the output -
# controller.py never reads it, only arm_state.
ch_high_aux = make_channels()
ch_high_aux[CH_AUX1] = CRSF_MAX
out = c.apply(ch_high_aux)
check("raw Aux1=high input is ignored while not armed", out[CH_AUX1] == CRSF_MIN)


print("\nTrackController - CH5 follows ARMED only (no LOCKED mirror on this branch)")
c, target, arm = new_controller()
target.publish(ex=0.1, ey=0.1, ex_rate=0.0, ey_rate=0.0)   # LOCKED, not armed
out = c.apply(make_channels())
check("CH5 stays LOW on LOCKED alone - arming still needs the Aux1 latch",
     out[CH_AUX1] == CRSF_MIN)
check("pilot keeps full manual roll/pitch/throttle while only LOCKED",
     out[CH_ROLL] == 1700 and out[CH_PITCH] == 300 and out[CH_THROTTLE] == 500)

arm.set(True)
out = c.apply(make_channels())
check("CH5 goes HIGH once ARMED", out[CH_AUX1] == CRSF_MAX)

arm.set(False)   # pilot lowered Aux1 - on this branch that disarms
out = c.apply(make_channels())
check("CH5 drops back LOW when disarmed - not one-way",
     out[CH_AUX1] == CRSF_MIN)
check("roll/pitch return to full pilot passthrough after disarm",
     out[CH_ROLL] == 1700 and out[CH_PITCH] == 300)


print("\nTrackController - ARMED, SEARCHING (no target)")
c, target, arm = new_controller()
arm.set(True)
out = c.apply(make_channels())
check("Aux1 forced high once armed", out[CH_AUX1] == CRSF_MAX)
check("roll neutral while searching", out[CH_ROLL] == CRSF_MID)
check("pitch neutral while searching", out[CH_PITCH] == CRSF_MID)
check("throttle is left as a raw pilot passthrough while searching",
     out[CH_THROTTLE] == 500)
check("pilot's roll/pitch stick values are ignored, but throttle isn't",
     out[CH_ROLL] != 1700 and out[CH_PITCH] != 300)


print("\nTrackController - ARMED, actively tracking")
c, target, arm = new_controller()
arm.set(True)
target.publish(ex=0.5, ey=-0.3, ex_rate=0.0, ey_rate=0.0)
out = c.apply(make_channels())
check("throttle stays a raw pilot passthrough while actively tracking too",
     out[CH_THROTTLE] == 500)
check("Aux4/Aux6 still pass straight through while actively tracking",
     out[CH_AUX4] == 777 and out[CH_AUX6] == 1234)
check("positive horizontal error pushes roll away from centre",
     out[CH_ROLL] != CRSF_MID)
check("negative vertical error pushes pitch away from centre",
     out[CH_PITCH] != CRSF_MID)
check("roll/pitch are a full override, not the pilot's stick values",
     out[CH_ROLL] != 1700 and out[CH_PITCH] != 300)


print("\nTrackController - live PID gains from GainState")
c, target, arm, gain = new_controller(with_gains=True)
arm.set(True)
target.publish(ex=0.5, ey=0.0, ex_rate=0.0, ey_rate=0.0)   # horizontal error only
gain.publish(dict.fromkeys(GainState.KEYS, 0.0))            # every gain -> 0
out = c.apply(make_channels())
check("all gains zero -> roll collapses to centre despite a live error",
     out[CH_ROLL] == CRSF_MID)

gain.publish({**dict.fromkeys(GainState.KEYS, 0.0), "roll_kp": 400.0})
out = c.apply(make_channels())
check("raising roll_kp live -> roll immediately drives off centre",
     out[CH_ROLL] != CRSF_MID)
check("pitch, with its gains still zero, stays centred",
     out[CH_PITCH] == CRSF_MID)


print("\nTrackController - gain_state=None keeps the config defaults")
c, target, arm = new_controller()   # gain_state=None, the default
arm.set(True)
target.publish(ex=0.5, ey=-0.3, ex_rate=0.0, ey_rate=0.0)
out = c.apply(make_channels())
check("armed+tracking still drives roll/pitch on the baked-in gains",
     out[CH_AUX1] == CRSF_MAX and out[CH_ROLL] != CRSF_MID
     and out[CH_PITCH] != CRSF_MID)


print("\nTrackController - stale target counts as no target")
from config import VISION_TIMEOUT
c, target, arm = new_controller()
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
