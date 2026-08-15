"""
CRSF serial bridge.

Owns the two serial ports and the forwarding threads. Knows nothing about
vision or control - it just calls controller.apply(channels) on each RC
frame. Pass PassThrough() for a transparent bridge.
"""

import sys
import threading
import time

import serial

from config import PORT_UP, PORT_DOWN, BAUD, TYPE_RC_CHANNELS
from crsf_protocol import (unpack_channels, pack_channels, build_frame,
                           parse_frames)
from state import Stats


class CrsfBridge:
    def __init__(self, controller, port_up=PORT_UP, port_down=PORT_DOWN,
                 baud_up=BAUD, baud_down=BAUD, on_channels=None):
        """
        controller   - object with .apply(channels) -> channels
        on_channels  - optional callback(raw_channels, out_channels) for
                       logging/display. raw is what the receiver sent,
                       out is what got forwarded to the FC.
        """
        self.controller = controller
        self.on_channels = on_channels
        self.stats = Stats()

        self.up = serial.Serial(port_up, baudrate=baud_up, timeout=0.02)
        self.down = serial.Serial(port_down, baudrate=baud_down, timeout=0.02)

        self._threads = []
        self._running = False

    # ----------------------------------------------------------------
    def start(self):
        self._running = True
        self._threads = [
            threading.Thread(target=self._pump_forward, daemon=True),
            threading.Thread(target=self._pump_telemetry, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        self._running = False
        time.sleep(0.05)
        try:
            self.up.close()
            self.down.close()
        except Exception:
            pass

    # ----------------------------------------------------------------
    def _pump_forward(self):
        """Receiver -> controller -> flight controller."""
        buf = bytearray()
        while self._running:
            try:
                chunk = self.up.read(self.up.in_waiting or 1)
                if chunk:
                    buf.extend(chunk)

                for ftype, payload, raw in parse_frames(buf):
                    if ftype is None:
                        self.stats.crc_err += 1
                        continue

                    if ftype == TYPE_RC_CHANNELS and len(payload) >= 22:
                        self.stats.rc += 1
                        channels = unpack_channels(payload[:22])
                        raw = list(channels)   # controller.apply() may mutate in place
                        channels = self.controller.apply(channels)
                        self.down.write(
                            build_frame(TYPE_RC_CHANNELS,
                                        pack_channels(channels)))
                        if self.on_channels:
                            self.on_channels(raw, channels)
                    else:
                        self.stats.other += 1
                        self.down.write(raw)

            except Exception as exc:
                print(f"\n[bridge/forward] {exc}", file=sys.stderr)
                time.sleep(0.05)

    def _pump_telemetry(self):
        """Flight controller -> receiver. Always transparent."""
        while self._running:
            try:
                data = self.down.read(self.down.in_waiting or 1)
                if data:
                    self.up.write(data)
                    self.stats.bytes_telemetry += len(data)
            except Exception:
                time.sleep(0.01)
