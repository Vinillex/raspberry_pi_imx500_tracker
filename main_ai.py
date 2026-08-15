#!/usr/bin/env python3
"""
CRSF bridge + IMX500 detection preview.

Wires the modules together:

    vision.Camera  ->  overlay (draw boxes)
    bridge.CrsfBridge  ->  controller.TrackController

The bridge runs on background threads; the vision loop owns the main
thread. Detections are display-only for now - nothing is tracked or
locked yet, so the controller never has a target to correct toward.

    python3 main_ai.py
    python3 main_ai.py --no-display
"""

import argparse
import time

from config import (PORT_UP, PORT_DOWN, BAUD, CH_AUX6,
                    RC_TIMEOUT, ZOOM_MIN, ZOOM_MAX, LOCK_CH, LOCK_CH_MIN,
                    ARM_CH, ARM_CH_MIN, RESCUE_CH, RESCUE_CH_MIN,
                    GREEN, ORANGE, RED, BLUE)
from crsf_protocol import crsf_to_range
from state import TargetState, ChannelState, ArmState, RescueState
from bridge import CrsfBridge
from controller import TrackController
from tracker import AuxLock, ArmLatch, GpsRescueLatch, ErrorTracker
from vision import auto_zoom_factor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--up", default=PORT_UP)
    ap.add_argument("--down", default=PORT_DOWN)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--no-display", action="store_true")
    args = ap.parse_args()

    target = TargetState()
    arm_state = ArmState()
    rescue_state = RescueState()
    controller = TrackController(target, arm_state, rescue_state)
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
    gps_latch = GpsRescueLatch()
    armed = False            # sticky once True; see tracker.ArmLatch
    zoom = ZOOM_MIN           # current zoom factor; Aux6-driven until armed,
                              # then auto_zoom_factor() takes over
    fps = 0.0
    prev_t = time.monotonic()

    if not args.no_display:
        import overlay

    print("Running. Ctrl+C to stop.\n")

    try:
        while True:
            frame, metadata = camera.capture()
            detections = camera.detections(metadata)

            now = time.monotonic()
            dt = max(now - prev_t, 1e-3)
            prev_t = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

            input_ch, output_ch, stamp = channel_state.snapshot()
            fresh = input_ch is not None and time.monotonic() - stamp <= RC_TIMEOUT

            lock_on_switch = fresh and input_ch[LOCK_CH] >= LOCK_CH_MIN
            aux1_high = fresh and input_ch[ARM_CH] >= ARM_CH_MIN
            aux4_high = fresh and input_ch[RESCUE_CH] >= RESCUE_CH_MIN

            # Aux5 -> lock, but block ACQUIRING a new lock while Aux1
            # (ARM) or Aux4 (RESCUE) is already high - forces a clean
            # low state on both first. This does NOT drop an already-
            # established lock: that would break the arm sequence
            # itself, which is "raise Aux1 to its edge *while locked*".
            already_locked = aux_lock.locked
            lock_blocked = (aux1_high or aux4_high) and not already_locked
            lock_on = lock_on_switch and not lock_blocked

            # Once locked, sticks with the same object even if something
            # with higher confidence enters the frame. Once ARMED,
            # tracking keeps running regardless of Aux5/Aux1 - the
            # one-way latch means `armed` here is last frame's value,
            # which is already True by the time this could matter.
            track_enabled = lock_on or armed
            locked_box = aux_lock.update(detections, track_enabled)
            is_locked = lock_on and locked_box is not None

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
            # locked AND Aux4 is low counts (see tracker.ArmLatch). Once
            # armed, it's a one-way latch: nothing disarms it for the
            # rest of this run.
            armed = arm_latch.update(aux1_high, is_locked and not aux4_high)
            arm_state.set(armed)

            # Aux4 -> GPS rescue. Triggers permanently on 5s of continuous
            # SEARCHING (see tracker.GpsRescueLatch), or immediately if
            # Aux4 goes high WHILE ALREADY ARMED/SEARCHING - `armed`
            # covers both. Lowering Aux4 afterward undoes neither trigger.
            searching = armed and locked_box is None
            gps_rescue, countdown = gps_latch.update(searching, aux4_high and armed)
            rescue_state.set(gps_rescue)

            # Blocking-state labels, right of centre - only relevant
            # before arming; once armed the main status text covers it.
            error_lines = []
            if not armed:
                if lock_blocked:
                    if aux1_high:
                        error_lines.append("ARMED")
                    if aux4_high:
                        error_lines.append("GPS RESCUE")
                elif is_locked and aux4_high:
                    error_lines.append("GPS RESCUE")

            # Zoom: once armed, Aux6 has no effect any more - a closed
            # loop drives CH10 itself to keep the locked box's height at
            # TARGET_BOX_FRAC of the frame height (see auto_zoom_factor
            # for the deadband/step-limiting/edge-margin/SEARCHING
            # behaviour). Before armed, Aux6 drives it manually as before.
            if armed:
                zoom = auto_zoom_factor(zoom, locked_box, camera.size)
                camera.set_zoom(zoom)
            elif fresh:
                zoom = crsf_to_range(input_ch[CH_AUX6], ZOOM_MIN, ZOOM_MAX)
                camera.set_zoom(zoom)

            if gps_rescue:
                # Terminal state - once latched, stays shown forever
                # regardless of what armed/locked_box do afterward.
                box, box_color = None, BLUE
                text = "GPS RESCUE"
            elif armed:
                if locked_box is not None:
                    box, box_color = locked_box, RED
                    text = "ARMED"
                else:
                    # Object left the frame - drop the box rather than
                    # hold a stale one, and flag that we're waiting for
                    # it to come back. Resumes ARMED the instant it's
                    # matched again (aux_lock keeps trying every frame).
                    box, box_color = None, RED
                    text = "SEARCHING"
            else:
                if lock_on:
                    box, box_color = locked_box, ORANGE
                    # "LOCKED" only while the object is actually matched
                    # this frame - the moment it goes out of frame (even
                    # if we're still waiting to reacquire the same one),
                    # show NO OBJECT DETECTED rather than stale LOCKED.
                    text = "LOCKED" if is_locked else "NO OBJECT DETECTED"
                else:
                    best = max(detections, key=lambda d: d.conf, default=None)
                    box = best.box if best else None
                    box_color = GREEN
                    text = "DETECTED" if best else "DETECTING"

            if args.no_display:
                continue

            overlay.draw(frame, box, box_color, text, box_color,
                        (input_ch, output_ch, stamp), countdown=countdown,
                        error_lines=error_lines, fps=fps)
            overlay.show(frame)

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
