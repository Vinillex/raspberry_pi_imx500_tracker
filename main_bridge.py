#!/usr/bin/env python3
"""
Plain CRSF bridge - no camera, no control law.

Forwards receiver -> flight controller transparently and prints channels.
Use this to verify the serial path independently of the vision stack.

    python3 main_bridge.py
    python3 main_bridge.py --show 16 --labels
    python3 main_bridge.py --down /dev/ttyAMA1 --baud-down 115200
"""

import argparse
import time

from config import PORT_UP, PORT_DOWN, BAUD, CH_NAMES
from bridge import CrsfBridge
from controller import PassThrough


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--up", default=PORT_UP)
    ap.add_argument("--down", default=PORT_DOWN)
    ap.add_argument("--baud-up", type=int, default=BAUD)
    ap.add_argument("--baud-down", type=int, default=BAUD)
    ap.add_argument("--show", type=int, default=10, metavar="1-16",
                    help="how many channels to print (default 10)")
    ap.add_argument("--labels", action="store_true",
                    help="print a channel-name header")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    state = {"last": 0.0, "header": False}

    def on_channels(raw, channels):
        if args.quiet:
            return
        now = time.time()
        if now - state["last"] < 0.2:
            return
        state["last"] = now

        n = min(args.show, len(channels))
        if args.labels and not state["header"]:
            print("       " + " ".join(f"{CH_NAMES[i]:>5s}" for i in range(n)))
            state["header"] = True
        vals = " ".join(f"{c:5d}" for c in channels[:n])
        print(f"\rCH:    {vals}", end="", flush=True)

    print(f"Upstream:   {args.up} @ {args.baud_up}")
    print(f"Downstream: {args.down} @ {args.baud_down}")
    print("Mode:       pass-through\n")

    bridge = CrsfBridge(PassThrough(), args.up, args.down,
                        args.baud_up, args.baud_down,
                        on_channels=on_channels)
    bridge.start()
    print("Bridging. Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        print(f"\n\nStopped. {bridge.stats}")


if __name__ == "__main__":
    main()
