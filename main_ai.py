#!/usr/bin/env python3
"""
CRSF bridge + IMX500 tracking, arming and GPS-rescue.

Wires the modules together:

    vision.Camera  ->  tracker.AuxLock / ErrorTracker  ->  state.TargetState
    bridge.CrsfBridge  ->  controller.TrackController

The bridge runs on background threads; the vision loop owns the main
thread. Aux5 locks onto a detection, Aux1 arms (edge-triggered, requires
LOCKED first, one-way), and controller.py takes over roll/pitch/throttle
entirely once armed - see README.md's "Safety gates" section. For bench
testing, lowering Aux1 while ARMED/GPS_RESCUE triggers DISABLED - a
one-way kill switch (tracker.DisableLatch) that forces CH5/CH8 low and
stops everything else, for the rest of the run.

    python3 main_ai.py
    python3 main_ai.py --no-display
"""

import argparse
import time

from config import (PORT_UP, PORT_DOWN, BAUD, CH_AUX6, FPS_ALPHA,
                    RC_TIMEOUT, ZOOM_MIN, ZOOM_MAX, LOCK_CH, LOCK_CH_MIN,
                    ARM_CH, ARM_CH_MIN, RESCUE_CH, RESCUE_CH_MIN,
                    GREEN, ORANGE, RED, BLUE, BLACK)
from crsf_protocol import crsf_to_range
from state import TargetState, ChannelState, ArmState, RescueState, DisableState
from bridge import CrsfBridge
from controller import TrackController
from tracker import AuxLock, ArmLatch, GpsRescueLatch, DisableLatch, ErrorTracker
from vision import auto_zoom_factor


def resolve_lock(aux_lock, detections, lock_on_switch, aux1_high, aux4_high, armed):
    """Aux5 -> lock, but block ACQUIRING a new lock while Aux1 (ARM) or
    Aux4 (RESCUE) is already high - forces a clean low state on both
    first. This does NOT drop an already-established lock: that would
    break the arm sequence itself, which is "raise Aux1 to its edge
    *while locked*".

    Once locked, sticks with the same object even if something with
    higher confidence enters the frame. Once ARMED, tracking keeps
    running regardless of Aux5/Aux1 - the one-way latch means `armed`
    here is last frame's value, which is already True by the time this
    could matter.

    Returns (lock_on, locked_box, is_locked, lock_blocked)."""
    already_locked = aux_lock.locked
    lock_blocked = (aux1_high or aux4_high) and not already_locked
    lock_on = lock_on_switch and not lock_blocked

    track_enabled = lock_on or armed
    locked_box = aux_lock.update(detections, track_enabled)
    is_locked = lock_on and locked_box is not None

    return lock_on, locked_box, is_locked, lock_blocked


def resolve_zoom(armed, fresh, input_ch, locked_box, camera, zoom):
    """Once armed, Aux6 has no effect any more - a closed loop drives
    CH10 itself to keep the locked box's height at TARGET_BOX_FRAC of
    the frame height (see vision.auto_zoom_factor for the deadband/
    step-limiting/edge-margin/SEARCHING behaviour). Before armed, Aux6
    drives it manually as before.

    Returns the new zoom factor; also calls camera.set_zoom() as a
    side effect, same as the inline code this replaces."""
    if armed:
        zoom = auto_zoom_factor(zoom, locked_box, camera.size)
        camera.set_zoom(zoom)
    elif fresh:
        zoom = crsf_to_range(input_ch[CH_AUX6], ZOOM_MIN, ZOOM_MAX)
        camera.set_zoom(zoom)
    return zoom


def select_overlay_state(disabled, gps_rescue, armed, locked_box, lock_on,
                        is_locked, detections):
    """Decide what the overlay should show this frame, in priority
    order: DISABLED (terminal - bench-test kill switch, tracker.
    DisableLatch, overrides everything else forever once triggered) >
    GPS_RESCUE (terminal - once latched, stays shown forever regardless
    of what armed/locked_box do afterward) > ARMED/SEARCHING (object
    left the frame -> drop the box rather than hold a stale one;
    resumes ARMED the instant it's matched again, since aux_lock keeps
    trying every frame) > LOCKED/NO OBJECT DETECTED ("LOCKED" only
    while the object is actually matched this frame) > DETECTING/
    DETECTED.

    Returns (box, box_color, text)."""
    if disabled:
        return None, BLACK, "DISABLED"

    if gps_rescue:
        return None, BLUE, "GPS RESCUE"

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
    rescue_state = RescueState()
    disable_state = DisableState()
    controller = TrackController(target, arm_state, rescue_state, disable_state)
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
    disable_latch = DisableLatch()
    armed = False            # sticky once True; see tracker.ArmLatch
    zoom = ZOOM_MIN           # current zoom factor; Aux6-driven until armed,
                              # then auto_zoom_factor() takes over
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

            lock_on_switch = fresh and input_ch[LOCK_CH] >= LOCK_CH_MIN
            aux1_high = fresh and input_ch[ARM_CH] >= ARM_CH_MIN
            aux4_high = fresh and input_ch[RESCUE_CH] >= RESCUE_CH_MIN

            lock_on, locked_box, is_locked, lock_blocked = resolve_lock(
                aux_lock, detections, lock_on_switch, aux1_high, aux4_high, armed)

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

            # Aux1 low while ARMED/GPS_RESCUE -> DISABLED: a deliberate
            # bench-test kill switch (see tracker.DisableLatch). One-way,
            # terminal - once this fires, nothing else in this loop or in
            # controller.py has any further effect for the rest of this run.
            disabled = disable_latch.update(aux1_high, armed or gps_rescue)
            disable_state.set(disabled)

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

            if disabled:
                countdown = None
                error_lines = []

            zoom = resolve_zoom(armed, fresh, input_ch, locked_box, camera, zoom)

            box, box_color, text = select_overlay_state(
                disabled, gps_rescue, armed, locked_box, lock_on, is_locked,
                detections)

            if args.no_display:
                continue

            overlay.draw(frame, box, box_color, text, box_color,
                        (input_ch, output_ch, stamp), countdown=countdown,
                        error_lines=error_lines, fps=fps)
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
