"""
Minimal OpenCV drawing.

A box around whatever is being shown (green for a plain detection,
orange for an Aux5 lock), a top-centre status line, and a channel-wise
readout at the bottom of what came in from the receiver and what went
out to the flight controller. Keeping this separate means the control
path has no dependency on cv2 and --no-display can skip this module
entirely. All the decisions (what box, what text, what colour) are
made by the caller - this module only draws.
"""

import time

import cv2

from config import WHITE, RED, GREEN, RC_TIMEOUT, CH_NAMES, CH_AUX6

FONT = cv2.FONT_HERSHEY_SIMPLEX
N_SHOWN = CH_AUX6 + 1   # CH1-10 (AETR + Aux1-6); CH11-16 not shown


def draw(frame, box, box_color, status_text, status_color, channel_snapshot,
        countdown=None, error_lines=None, fps=None):
    """Draw the subject box, status text, CRSF channel readout and (if
    given) the GPS-rescue countdown, blocking-state labels and frame
    rate, in place.

    box            - (x, y, w, h) or None
    channel_snapshot - whatever ChannelState.snapshot() returned:
                        (input_channels, output_channels, stamp)
    countdown      - seconds remaining before GPS_RESCUE triggers, or
                      None to hide it
    error_lines    - list of labels (e.g. ["ARMED", "GPS RESCUE"]) to
                      stack at right-centre, one per line; None/empty
                      to hide
    fps            - current frame rate, or None to hide it
    """
    if box is not None:
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)

    _draw_status(frame, status_text, status_color)
    _draw_countdown(frame, countdown)
    _draw_error(frame, error_lines)
    _draw_fps(frame, fps)
    _draw_channels(frame, channel_snapshot)
    return frame


def _draw_status(frame, text, color):
    w = frame.shape[1]
    (tw, _), _ = cv2.getTextSize(text, FONT, 0.7, 2)
    x = (w - tw) // 2
    cv2.putText(frame, text, (x, 30), FONT, 0.7, color, 2)


def _draw_countdown(frame, countdown):
    if countdown is None:
        return
    w = frame.shape[1]
    text = f"{countdown:.1f}"
    (tw, _), _ = cv2.getTextSize(text, FONT, 0.7, 2)
    cv2.putText(frame, text, (w - tw - 10, 30), FONT, 0.7, RED, 2)


def _draw_error(frame, lines):
    if not lines:
        return
    h, w = frame.shape[:2]
    y = h // 2
    for line in lines:
        (tw, th), _ = cv2.getTextSize(line, FONT, 0.55, 2)
        cv2.putText(frame, line, (w - tw - 10, y), FONT, 0.55, RED, 2)
        y += th + 14


def _draw_fps(frame, fps):
    if fps is None:
        return
    cv2.putText(frame, f"{fps:.0f}", (10, 26), FONT, 0.7, GREEN, 2)


def _draw_channels(frame, channel_snapshot):
    h = frame.shape[0]
    input_ch, output_ch, stamp = channel_snapshot
    stale = input_ch is None or (time.monotonic() - stamp) > RC_TIMEOUT

    if stale:
        cv2.putText(frame, "RX: NA", (10, h - 12), FONT, 0.5, RED, 1)
        return

    labels = "     " + " ".join(f"{CH_NAMES[i]:>5s}" for i in range(N_SHOWN))
    in_str = "IN:  " + " ".join(f"{c:5d}" for c in input_ch[:N_SHOWN])
    out_str = "OUT: " + " ".join(f"{c:5d}" for c in output_ch[:N_SHOWN])
    cv2.putText(frame, labels, (10, h - 50), FONT, 0.45, WHITE, 1)
    cv2.putText(frame, in_str, (10, h - 32), FONT, 0.45, WHITE, 1)
    cv2.putText(frame, out_str, (10, h - 14), FONT, 0.45, WHITE, 1)


def show(frame, window="rpi_ai"):
    cv2.imshow(window, frame)
    return cv2.waitKey(1) & 0xFF


def destroy():
    cv2.destroyAllWindows()
