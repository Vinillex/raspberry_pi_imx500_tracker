"""
Thread-safe handoff between the vision loop and the CRSF bridge thread.

The vision loop calls publish() / invalidate().
The bridge thread calls snapshot().
Nothing else should touch the private fields.
"""

import threading
import time

from config import ROLL_KP, ROLL_KI, ROLL_KD, PITCH_KP, PITCH_KI, PITCH_KD


class TargetState:
    def __init__(self):
        self._lock = threading.Lock()
        self._ex = 0.0
        self._ey = 0.0
        self._ex_rate = 0.0
        self._ey_rate = 0.0
        self._locked = False
        self._stamp = 0.0

    def publish(self, ex, ey, ex_rate, ey_rate):
        """Called by vision when a locked target has been measured."""
        with self._lock:
            self._ex = ex
            self._ey = ey
            self._ex_rate = ex_rate
            self._ey_rate = ey_rate
            self._locked = True
            self._stamp = time.monotonic()

    def snapshot(self):
        """Returns (ex, ey, ex_rate, ey_rate, locked, stamp)."""
        with self._lock:
            return (self._ex, self._ey, self._ex_rate, self._ey_rate,
                    self._locked, self._stamp)

    def invalidate(self):
        """Called when the target is lost or released. Stops AI output."""
        with self._lock:
            self._locked = False


class ChannelState:
    """Thread-safe handoff of the latest CRSF channel values.

    The bridge thread calls publish() on every RC-channels frame.
    The display loop calls snapshot() to read it for the overlay.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._input = None
        self._output = None
        self._stamp = 0.0

    def publish(self, input_channels, output_channels):
        """input_channels is what the receiver sent, output_channels is
        what got forwarded to the flight controller (post-correction)."""
        with self._lock:
            self._input = list(input_channels)
            self._output = list(output_channels)
            self._stamp = time.monotonic()

    def snapshot(self):
        """Returns (input_channels, output_channels, stamp).
        input_channels is None if nothing has been received yet."""
        with self._lock:
            return self._input, self._output, self._stamp


class _BoolFlag:
    """Thread-safe boolean flag - the shared shape behind ArmState
    below (kept as a base class so more flag-shaped state can reuse it)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._value = False

    def set(self, value):
        with self._lock:
            self._value = value

    def get(self):
        with self._lock:
            return self._value


class ArmState(_BoolFlag):
    """Thread-safe boolean flag for the ARMED latch.

    The vision/main thread calls set() whenever tracker.ArmLatch's
    decision changes. The bridge thread calls get() every RC frame to
    decide whether to force the arm channel high.
    """


class GainState:
    """Thread-safe handoff of the six live PID gains from the tuning
    loop to the control law.

    The vision/main thread (tracker.GainTuner) calls publish() every
    frame with the current gains. The bridge thread
    (controller.TrackController) calls roll() / pitch() just before each
    PID update. Starts at the config defaults so the controller always
    has sane gains even before the first publish().
    """

    KEYS = ("roll_kp", "roll_ki", "roll_kd",
            "pitch_kp", "pitch_ki", "pitch_kd")

    _DEFAULTS = {
        "roll_kp": ROLL_KP, "roll_ki": ROLL_KI, "roll_kd": ROLL_KD,
        "pitch_kp": PITCH_KP, "pitch_ki": PITCH_KI, "pitch_kd": PITCH_KD,
    }

    def __init__(self):
        self._lock = threading.Lock()
        self._g = dict(self._DEFAULTS)

    def publish(self, gains):
        """gains is a dict with (at least) the six KEYS."""
        with self._lock:
            for k in self.KEYS:
                if k in gains:
                    self._g[k] = float(gains[k])

    def snapshot(self):
        """Returns a plain dict copy of all six gains."""
        with self._lock:
            return dict(self._g)

    def roll(self):
        """Returns (kp, ki, kd) for the roll axis."""
        with self._lock:
            return self._g["roll_kp"], self._g["roll_ki"], self._g["roll_kd"]

    def pitch(self):
        """Returns (kp, ki, kd) for the pitch axis."""
        with self._lock:
            return self._g["pitch_kp"], self._g["pitch_ki"], self._g["pitch_kd"]


class Stats:
    """Frame counters. Integer increments are atomic enough for display."""

    def __init__(self):
        self.rc = 0
        self.other = 0
        self.crc_err = 0
        self.bytes_telemetry = 0

    def __str__(self):
        return (f"rc:{self.rc} other:{self.other} "
                f"crc_err:{self.crc_err} telem:{self.bytes_telemetry}")
