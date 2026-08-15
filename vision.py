"""
IMX500 camera setup and detection.

Isolates every picamera2 / libcamera import so the rest of the project can
be imported and tested on a machine without a camera.
"""

import sys

from config import (MODEL, TARGET_CLASS, THRESHOLD, MAX_DETECTIONS,
                    MAIN_SIZE, HFLIP, VFLIP, ZOOM_MIN, ZOOM_MAX,
                    TARGET_BOX_FRAC, AUTO_ZOOM_DEADBAND, AUTO_ZOOM_KP,
                    AUTO_ZOOM_MAX_STEP, AUTO_ZOOM_EDGE_MARGIN_FRAC)


def box_center(box):
    x, y, w, h = box
    return (x + w / 2.0, y + h / 2.0)


def auto_zoom_factor(current_factor, box, frame_size,
                     target_frac=TARGET_BOX_FRAC,
                     deadband=AUTO_ZOOM_DEADBAND,
                     kp=AUTO_ZOOM_KP, max_step=AUTO_ZOOM_MAX_STEP,
                     edge_margin_frac=AUTO_ZOOM_EDGE_MARGIN_FRAC):
    """Proportional zoom control, height-based: nudges `current_factor`
    so box height / frame height moves toward `target_frac`. Within
    +/-`deadband` of the target (e.g. 40-60%), holds still rather than
    correcting. Each call moves by at most `max_step`, so zoom eases
    in/out slowly instead of jumping and overshooting.

    Also holds still if `box` is within `edge_margin_frac` of any frame
    edge - the crop is always centred on the frame (not the box), so
    zooming further in that situation risks cropping the target out
    entirely rather than framing it better.

    `box=None` means nothing to track (SEARCHING) - zoom out completely
    (ZOOM_MIN) for maximum FOV to help reacquire, rather than holding."""
    if box is None:
        return ZOOM_MIN

    frame_w, frame_h = frame_size
    x, y, bw, bh = box

    margin_x = edge_margin_frac * frame_w
    margin_y = edge_margin_frac * frame_h
    near_edge = (x < margin_x or y < margin_y
                or (frame_w - (x + bw)) < margin_x
                or (frame_h - (y + bh)) < margin_y)
    if near_edge:
        return current_factor

    frac = bh / frame_h
    error = target_frac - frac
    if abs(error) <= deadband:
        return current_factor

    step = max(-max_step, min(max_step, kp * error))
    return max(ZOOM_MIN, min(ZOOM_MAX, current_factor + step))


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
        self._full_crop = self.picam2.camera_properties.get("ScalerCropMaximum")

        # Needed to sync the network's own analysis crop to the zoom - without
        # this, ScalerCrop alone only crops what's *displayed*; the IMX500
        # accelerator keeps running inference on the full, un-zoomed sensor
        # image, so it would still detect things outside the zoomed-in view.
        self._roi_supported = hasattr(self.imx500, "set_inference_roi_abs")
        if not self._roi_supported:
            print("IMX500: set_inference_roi_abs() not available - zoom "
                  "will crop the display but not restrict detection",
                  file=sys.stderr)

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

    def set_zoom(self, factor):
        """Digital zoom via sensor crop, synced to the IMX500 inference ROI
        so detection only runs on the zoomed-in region - not the full
        frame. factor=1.0 is full FOV; larger values crop in tighter
        around the frame centre."""
        if not self._full_crop:
            return
        factor = max(1.0, factor)
        fx, fy, fw, fh = self._full_crop
        w = int(fw / factor)
        h = int(fh / factor)
        x = fx + (fw - w) // 2
        y = fy + (fh - h) // 2
        crop = (x, y, w, h)

        try:
            self.picam2.set_controls({"ScalerCrop": crop})
        except Exception:
            pass

        if self._roi_supported:
            try:
                self.imx500.set_inference_roi_abs(crop)
            except Exception:
                pass

    def stop(self):
        try:
            self.picam2.stop()
        except Exception:
            pass
