"""
Thread-safe handoff between the vision loop and the CRSF bridge thread.

The vision loop calls publish() / invalidate().
The bridge thread calls snapshot().
Nothing else should touch the private fields.
"""

import threading
import time


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
    """Thread-safe boolean flag - the shared shape behind ArmState and
    RescueState below, which differ only in what the flag means."""

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


class RescueState(_BoolFlag):
    """Thread-safe boolean flag for the GPS-rescue latch.

    The vision/main thread calls set() whenever tracker.GpsRescueLatch's
    decision changes. The bridge thread calls get() every RC frame to
    decide whether to force the rescue channel (CH8/Aux4) high.
    """


class DisableState(_BoolFlag):
    """Thread-safe boolean flag for the DISABLED kill-switch latch.

    The vision/main thread calls set() whenever tracker.DisableLatch's
    decision changes. The bridge thread calls get() every RC frame to
    decide whether to force CH5/CH8 low and stop driving anything else.
    """


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
