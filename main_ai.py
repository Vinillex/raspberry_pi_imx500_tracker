#!/usr/bin/env python3
"""
CRSF bridge + IMX500 tracking and arming.

Wires the modules together:

    vision.Camera  ->  tracker.AuxLock / ErrorTracker  ->  state.TargetState
    bridge.CrsfBridge  ->  controller.TrackController

The bridge runs on background threads; the vision loop owns the main
thread. Aux5 locks onto a detection, Aux1 arms (edge-triggered, requires
LOCKED first), and controller.py takes over roll/pitch entirely once
armed - see README.md's "Safety gates" section. On this static-testing
branch arming is not one-way: lowering Aux1 disarms (drops CH5, disarms
the FC), ready for the next tuning pass. Aux2/Aux3/Aux4 select a PID
gain (any state) and Aux6 (a spring-return wheel) ramps it live while
ARMED - see tracker.GainTuner.

    python3 main_ai.py
    python3 main_ai.py --no-display
"""

import argparse
import time

from config import (PORT_UP, PORT_DOWN, BAUD, FPS_ALPHA,
                    RC_TIMEOUT, LOCK_CH, LOCK_CH_MIN,
                    ARM_CH, ARM_CH_MIN,
                    CH_AUX2, CH_AUX3, CH_AUX4, CH_AUX6,
                    CRSF_MID, CRSF_MIN,
                    GREEN, ORANGE, RED)
from state import TargetState, ChannelState, ArmState, GainState
from bridge import CrsfBridge
from controller import TrackController
from tracker import AuxLock, ArmLatch, GainTuner, ErrorTracker


def resolve_lock(aux_lock, detections, lock_on_switch, aux1_high, armed):
    """Aux5 -> lock, but block ACQUIRING a new lock while Aux1 (ARM) is
    already high - forces a clean low state first. This does NOT drop an
    already-established lock: that would break the arm sequence itself,
    which is "raise Aux1 to its edge *while locked*".

    Once locked, sticks with the same object even if something with
    higher confidence enters the frame. Once ARMED, tracking keeps
    running regardless of Aux5/Aux1 - the one-way latch means `armed`
    here is last frame's value, which is already True by the time this
    could matter.

    Returns (lock_on, locked_box, is_locked, lock_blocked)."""
    already_locked = aux_lock.locked
    lock_blocked = aux1_high and not already_locked
    lock_on = lock_on_switch and not lock_blocked

    track_enabled = lock_on or armed
    locked_box = aux_lock.update(detections, track_enabled)
    is_locked = lock_on and locked_box is not None

    return lock_on, locked_box, is_locked, lock_blocked


def select_overlay_state(armed, locked_box, lock_on, is_locked, detections):
    """Decide what the overlay should show this frame, in priority
    order: ARMED/SEARCHING (object left the frame -> drop the box rather
    than hold a stale one; resumes ARMED the instant it's matched again,
    since aux_lock keeps trying every frame) > LOCKED/NO OBJECT DETECTED
    ("LOCKED" only while the object is actually matched this frame) >
    DETECTING/DETECTED (the state the gain panel is shown in).

    Returns (box, box_color, text)."""
    if armed:
        if locked_box is not None:
            return locked_box, RED, "ARMED"
        return None, RED, "SEARCHING"

    if lock_on:
        text = "LOCKED" if is_locked else "NO OBJECT DETECTED"
        return locked_box, ORANGE, text

    best = max(detections, key=lambda d: d.conf, default=None)
    box = best.box if best else None
    text = "DETECTED" if best else "DETECTING"
    return box, GREEN, text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--up", default=PORT_UP)
    ap.add_argument("--down", default=PORT_DOWN)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--no-display", action="store_true")
    args = ap.parse_args()

    target = TargetState()
    arm_state = ArmState()
    gain_state = GainState()
    controller = TrackController(target, arm_state, gain_state)
    channel_state = ChannelState()

    # Serial first - if the ports fail we should not start the camera.
    bridge = CrsfBridge(controller, args.up, args.down, args.baud, args.baud,
                        on_channels=channel_state.publish)
    bridge.start()
    print(f"Bridge: {args.up} -> {args.down} @ {args.baud}")

    # Camera imports happen inside vision.Camera, so a missing camera
    # fails here rather than at module import time.
    from vision import Camera
    camera = Camera()
    aux_lock = AuxLock(size=camera.size)
    error_tracker = ErrorTracker(size=camera.size)
    arm_latch = ArmLatch()
    gain_tuner = GainTuner()
    armed = False            # follows Aux1 once locked; see tracker.ArmLatch
    fps = 0.0
    prev_t = time.monotonic()

    if not args.no_display:
        import overlay

    quit_hint = "" if args.no_display else ", or focus the video window and press q/Esc"
    print(f"Running. Ctrl+C to stop{quit_hint}.\n")

    try:
        while True:
            frame, metadata = camera.capture()
            detections = camera.detections(metadata)

            now = time.monotonic()
            dt = max(now - prev_t, 1e-3)
            prev_t = now
            fps = (1 - FPS_ALPHA) * fps + FPS_ALPHA * (1.0 / dt)

            input_ch, output_ch, stamp = channel_state.snapshot()
            fresh = input_ch is not None and time.monotonic() - stamp <= RC_TIMEOUT

            # Aux2/Aux3/Aux4 select a PID gain (any state); Aux6
            # (spring-return wheel) ramps it, but only while ARMED - in
            # DETECTING you just pick the gain. `armed` here is last
            # frame's value (recomputed below); a one-frame lag on the
            # wheel going live is imperceptible. Fall back to neutral
            # positions when RC is stale.
            aux2 = input_ch[CH_AUX2] if fresh else CRSF_MID
            aux3 = input_ch[CH_AUX3] if fresh else CRSF_MID
            aux4 = input_ch[CH_AUX4] if fresh else CRSF_MIN
            aux6 = input_ch[CH_AUX6] if fresh else CRSF_MID
            gains, selected_gain, aux6_centered = gain_tuner.update(
                aux2, aux3, aux4, aux6, dt, allow_ramp=armed)
            gain_state.publish(gains)

            aux1_high = fresh and input_ch[ARM_CH] >= ARM_CH_MIN
            # Aux5 is only honoured once Aux6 is centred - keeps a gain
            # from ramping while you set up the lock/arm sequence.
            lock_switch_raw = fresh and input_ch[LOCK_CH] >= LOCK_CH_MIN
            lock_on_switch = lock_switch_raw and aux6_centered

            lock_on, locked_box, is_locked, lock_blocked = resolve_lock(
                aux_lock, detections, lock_on_switch, aux1_high, armed)

            # Feed the box position into TargetState so controller.py's
            # PID loops (which run on the bridge thread) have something
            # to drive roll/pitch from once armed. No box -> invalidate,
            # so a stalled/absent target can't hold a stale correction.
            err = error_tracker.update(locked_box)
            if err is not None:
                target.publish(*err)
            else:
                target.invalidate()

            # Aux1 -> arm, but only an edge that happens while already
            # locked counts (see tracker.ArmLatch). On this branch it is
            # not one-way: lowering Aux1 disarms again, ready for the next
            # gain. controller.py drops CH5 (disarming the FC) to match.
            armed = arm_latch.update(aux1_high, is_locked)
            arm_state.set(armed)

            # Blocking-state label, right of centre - "ARMED" while a
            # new-lock attempt is blocked by Aux1 already being high.
            # Only reachable pre-arm (once armed the main status text
            # covers it).
            error_lines = []
            if not armed and lock_blocked and aux1_high:
                error_lines.append("ARMED")

            box, box_color, text = select_overlay_state(
                armed, locked_box, lock_on, is_locked, detections)

            # Gain readout: the full six-row panel while selecting
            # (DETECTING - not armed, no lock switch), or just the
            # selected gain + its live value while ARMED so you can watch
            # it move. Hidden in between (locked, not yet armed).
            detecting = not armed and not lock_on
            panel = gains if (detecting or armed) else None

            if args.no_display:
                continue

            overlay.draw(frame, box, box_color, text, box_color,
                        (input_ch, output_ch, stamp),
                        error_lines=error_lines, fps=fps,
                        gains=panel, selected_gain=selected_gain,
                        gains_compact=armed, aux6_centered=aux6_centered)
            key = overlay.show(frame)
            if key in (ord('q'), 27):   # 27 = Esc
                break

    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        if not args.no_display:
            overlay.destroy()
        bridge.stop()
        print(f"\nStopped. {bridge.stats}")


if __name__ == "__main__":
    main()
