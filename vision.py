"""
IMX500 camera setup and detection.

Isolates every picamera2 / libcamera import so the rest of the project can
be imported and tested on a machine without a camera.
"""

import sys

from config import (MODEL, TARGET_CLASS, THRESHOLD, MAX_DETECTIONS,
                    MAIN_SIZE, HFLIP, VFLIP)


def box_center(box):
    x, y, w, h = box
    return (x + w / 2.0, y + h / 2.0)


def nearest_idx(detections, point):
    """Index of the detection whose centre is closest to `point`,
    plus the squared distance. Returns (-1, None) if empty."""
    best, best_d2 = -1, None
    for i, d in enumerate(detections):
        cx, cy = box_center(d.box)
        d2 = (cx - point[0]) ** 2 + (cy - point[1]) ** 2
        if best_d2 is None or d2 < best_d2:
            best, best_d2 = i, d2
    return best, best_d2


class Detection:
    __slots__ = ("box", "conf")

    def __init__(self, box, conf):
        self.box = box
        self.conf = conf


class Camera:
    """Thin wrapper around Picamera2 + IMX500."""

    def __init__(self, model=MODEL, size=MAIN_SIZE):
        # Imported here so `import vision` does not fail on a dev machine.
        from libcamera import Transform
        from picamera2 import Picamera2
        from picamera2.devices import IMX500
        from picamera2.devices.imx500 import NetworkIntrinsics

        self.imx500 = IMX500(model)
        intrinsics = self.imx500.network_intrinsics
        if not intrinsics:
            intrinsics = NetworkIntrinsics()
            intrinsics.task = "object detection"
        elif intrinsics.task != "object detection":
            print("Loaded network is not an object detection model",
                  file=sys.stderr)
            sys.exit(1)
        intrinsics.update_with_defaults()
        self.intrinsics = intrinsics

        self.picam2 = Picamera2(self.imx500.camera_num)
        self.config = self.picam2.create_preview_configuration(
            main={"size": size, "format": "RGB888"},
            controls={"FrameRate": intrinsics.inference_rate},
            transform=Transform(hflip=HFLIP, vflip=VFLIP),
            buffer_count=12,
        )
        self.imx500.show_network_fw_progress_bar()
        self.picam2.start(self.config)
        self.size = size

    # ----------------------------------------------------------------
    def capture(self):
        """Returns (frame_ndarray, metadata)."""
        request = self.picam2.capture_request()
        metadata = request.get_metadata()
        frame = request.make_array("main").copy()
        request.release()
        return frame, metadata

    def detections(self, metadata, target_class=TARGET_CLASS,
                   threshold=THRESHOLD, limit=MAX_DETECTIONS):
        """Parse the sensor's inference output into Detection objects."""
        outs = self.imx500.get_outputs(metadata, add_batch=True)
        if outs is None:
            return []

        boxes, scores, classes = outs[0][0], outs[1][0], outs[2][0]
        result = []
        for box, score, cls in zip(boxes, scores, classes):
            if int(cls) != target_class or score < threshold:
                continue
            coords = self.imx500.convert_inference_coords(
                box, metadata, self.picam2)
            result.append(Detection(coords, float(score)))
        return result[:limit]

    def stop(self):
        try:
            self.picam2.stop()
        except Exception:
            pass
