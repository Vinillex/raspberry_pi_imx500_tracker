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

STATUS_SCALE = 0.7      # top-centre status text, top-right countdown, FPS
LABEL_SCALE = 0.55      # right-centre blocking-state labels
CHANNEL_SCALE = 0.45    # bottom channel readout
STALE_SCALE = 0.5       # "RX: NA"
TEXT_MARGIN = 10        # px from the frame edge for right-aligned text


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


def _text(frame, text, y, color, scale=STATUS_SCALE, align="center", thickness=2):
    """Measure `text` and draw it at height `y`, horizontally centred or
    right-aligned (with TEXT_MARGIN from the frame edge). Returns the
    text height, so callers stacking multiple lines can advance `y`."""
    w = frame.shape[1]
    (tw, th), _ = cv2.getTextSize(text, FONT, scale, thickness)
    x = (w - tw) // 2 if align == "center" else w - tw - TEXT_MARGIN
    cv2.putText(frame, text, (x, y), FONT, scale, color, thickness)
    return th


def _draw_status(frame, text, color):
    _text(frame, text, 30, color)


def _draw_countdown(frame, countdown):
    if countdown is None:
        return
    _text(frame, f"{countdown:.1f}", 30, RED, align="right")


def _draw_error(frame, lines):
    if not lines:
        return
    y = frame.shape[0] // 2
    for line in lines:
        th = _text(frame, line, y, RED, scale=LABEL_SCALE, align="right")
        y += th + 14


def _draw_fps(frame, fps):
    if fps is None:
        return
    cv2.putText(frame, f"{fps:.0f}", (10, 26), FONT, STATUS_SCALE, GREEN, 2)


def _draw_channels(frame, channel_snapshot):
    h = frame.shape[0]
    input_ch, output_ch, stamp = channel_snapshot
    stale = input_ch is None or (time.monotonic() - stamp) > RC_TIMEOUT

    if stale:
        cv2.putText(frame, "RX: NA", (10, h - 12), FONT, STALE_SCALE, RED, 1)
        return

    labels = "     " + " ".join(f"{CH_NAMES[i]:>5s}" for i in range(N_SHOWN))
    in_str = "IN:  " + " ".join(f"{c:5d}" for c in input_ch[:N_SHOWN])
    out_str = "OUT: " + " ".join(f"{c:5d}" for c in output_ch[:N_SHOWN])
    cv2.putText(frame, labels, (10, h - 50), FONT, CHANNEL_SCALE, WHITE, 1)
    cv2.putText(frame, in_str, (10, h - 32), FONT, CHANNEL_SCALE, WHITE, 1)
    cv2.putText(frame, out_str, (10, h - 14), FONT, CHANNEL_SCALE, WHITE, 1)


_window_ready = False


def show(frame, window="rpi_ai"):
    global _window_ready
    if not _window_ready:
        # WINDOW_NORMAL (resizable) is required before the fullscreen
        # property will actually take effect - the default AUTOSIZE
        # window cv2.imshow() would otherwise create ignores it.
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN,
                              cv2.WINDOW_FULLSCREEN)
        _window_ready = True

    cv2.imshow(window, frame)
    return cv2.waitKey(1) & 0xFF


def destroy():
    global _window_ready
    cv2.destroyAllWindows()
    _window_ready = False
