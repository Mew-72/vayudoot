"""Camera + photo logger + tiled person detection + geotagging + duplicate merging.

Photos are taken every `trigger_m` metres of travel (not on a timer), each saved with the drone's
position/attitude interpolated to the moment the frame was captured.
"""
import csv
import json
import math
import os
import queue
import threading
import time

import cv2
import numpy as np

from geo import dist_m
from geotag import camera_attitude, intrinsics_from_hfov, offset_to_latlon, pixel_to_ned_offset


# ---------------------------------------------------------------- camera
class Grabber(threading.Thread):
    """Reads the camera continuously and keeps only the newest frame (UVC cameras buffer old ones)."""

    def __init__(self, device, width, height, fps=15):
        super().__init__(daemon=True)
        # if isinstance(device, int) or str(device).startswith("/dev/video"):
        #     self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        # else:
        #     self.cap = cv2.VideoCapture(device)          # recorded video file or rtsp:// url
        # 
        # USing windows laptop camera
        if isinstance(device, int):
            backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_V4L2
            self.cap = cv2.VideoCapture(device, backend)
        
            # Fall back to OpenCV's automatically selected backend if needed.
            if not self.cap.isOpened():
                self.cap.release()
                self.cap = cv2.VideoCapture(device)
        elif str(device).startswith("/dev/video"):
            self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(device)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.opened = self.cap.isOpened()
        self.is_file = os.path.isfile(str(device))
        self._lock = threading.Lock()
        self._frame, self._t = None, 0.0

    def run(self):
        while True:
            ok, f = self.cap.read()
            if not ok:
                if self.is_file:                       # recorded video: loop it (for rehearsals without the camera)
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    time.sleep(0.03)
                else:
                    time.sleep(0.05)
                continue
            if self.is_file:
                time.sleep(0.03)                       # play at roughly real time
            t = time.monotonic()
            with self._lock:
                self._frame, self._t = f, t

    def latest(self):
        with self._lock:
            if self._frame is None:
                return None, 0.0
            return self._frame.copy(), self._t

    def age(self):
        with self._lock:
            return 999.0 if self._frame is None else time.monotonic() - self._t


# ---------------------------------------------------------------- detection
def _positions(length, tile, step):
    last = max(length - tile, 0)
    xs = list(range(0, last + 1, step))
    if xs[-1] != last:
        xs.append(last)
    return xs


def tiled_detect(model, img, tile=640, overlap=0.2, conf=0.3, classes=(0,)):
    """Run the detector on overlapping tiles at full resolution (people are small)."""
    H, W = img.shape[:2]
    step = max(1, int(tile * (1 - overlap)))
    boxes, scores = [], []
    for y0 in _positions(H, tile, step):
        for x0 in _positions(W, tile, step):
            crop = img[y0:y0 + tile, x0:x0 + tile]
            r = model.predict(crop, imgsz=tile, conf=conf, classes=list(classes), verbose=False)[0]
            for b in r.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                boxes.append([x1 + x0, y1 + y0, x2 - x1, y2 - y1])
                scores.append(float(b.conf))
    if not boxes:
        return []
    keep = cv2.dnn.NMSBoxes(boxes, scores, conf, 0.5)
    keep = np.array(keep).flatten()
    return [(boxes[i][0], boxes[i][1], boxes[i][0] + boxes[i][2], boxes[i][1] + boxes[i][3], scores[i]) for i in keep]


def load_detector(det_cfg):
    try:
        from ultralytics import YOLO
        return YOLO(det_cfg["weights"], task="detect")
    except Exception as e:                      # keep logging photos even if the model is missing
        print(f"[vision] detector disabled: {e}", flush=True)
        return None


# ---------------------------------------------------------------- merging duplicates
class Clusters:
    """Merge repeated sightings of the same person into one survivor."""

    def __init__(self, radius_m=3.0, confirm_hits=2, confirm_conf=0.6):
        self.radius, self.confirm_hits, self.confirm_conf = radius_m, confirm_hits, confirm_conf
        self.items = []
        self.lock = threading.Lock()

    def add(self, lat, lon, conf, photo):
        with self.lock:
            for c in self.items:
                if dist_m(c["lat"], c["lon"], lat, lon) <= self.radius:
                    n = c["hits"]
                    c["lat"] = (c["lat"] * n + lat) / (n + 1)
                    c["lon"] = (c["lon"] * n + lon) / (n + 1)
                    c["hits"] = n + 1
                    c["conf"] = max(c["conf"], conf)
                    c["photos"].append(photo)
                    c["confirmed"] = c["hits"] >= self.confirm_hits or c["conf"] >= self.confirm_conf
                    return c
            c = {"id": len(self.items) + 1, "lat": lat, "lon": lon, "hits": 1, "conf": conf, "photos": [photo],
                 "confirmed": conf >= self.confirm_conf}
            self.items.append(c)
            return c

    def snapshot(self):
        with self.lock:
            return [dict(c, photos=c["photos"][-3:]) for c in self.items]


# ---------------------------------------------------------------- the survey logger
class Survey:
    def __init__(self, fc, cfg, mission, out_dir, min_agl=4.0):
        self.fc, self.cfg, self.mission = fc, cfg, mission
        self.cam_cfg, self.det_cfg = cfg["camera"], cfg["detector"]
        self.trigger_m = mission["trigger_m"]
        self.latency = self.cam_cfg.get("latency_s", 0.15)
        self.min_agl = min_agl
        self.out = out_dir
        self.photos_dir = os.path.join(out_dir, "photos")
        self.annot_dir = os.path.join(out_dir, "annotated")
        os.makedirs(self.photos_dir, exist_ok=True)
        os.makedirs(self.annot_dir, exist_ok=True)

        self.grab = Grabber(self.cam_cfg["device"], self.cam_cfg["width"], self.cam_cfg["height"],
                            self.cam_cfg.get("fps", 15))
        self.detector = load_detector(self.det_cfg)
        self.clusters = Clusters()
        self.q = queue.Queue(maxsize=40)
        self.stop_evt = threading.Event()
        self.n = 0
        self.raw_dets = []
        self.track = []
        self.stats = {"photos": 0, "dropped": 0, "processed": 0, "raw_detections": 0, "last_ms": 0,
                      "camera_ok": self.grab.opened, "detector_on": self.detector is not None,
                      "stale_frames": 0, "camera_age_s": 999.0}
        self._last_annotated = None
        self.threads = []

    # ----- control
    def start(self):
        self.grab.start()
        for fn in (self._trigger_loop, self._worker, self._track_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self.threads.append(t)

    def stop(self):
        self.stop_evt.set()
        for t in self.threads:
            t.join(timeout=30)
        self._write_outputs()

    # ----- threads
    def _trigger_loop(self):
        last_ll = None
        while not self.stop_evt.is_set():
            time.sleep(0.05)
            s = self.fc.snapshot()
            if s["lat"] is None or s["agl"] < self.min_agl:
                continue
            if last_ll is not None and dist_m(last_ll[0], last_ll[1], s["lat"], s["lon"]) < self.trigger_m:
                continue
            frame, tf = self.grab.latest()
            if frame is None:
                continue
            if time.monotonic() - tf > 0.5:             # camera froze: never log an old frame with a new position
                self.stats["stale_frames"] += 1
                continue
            st = self.fc.state_at(tf - self.latency)
            if st is None:
                continue
            lat, lon, agl, roll, pitch, yaw = st
            meta = {"t": tf, "lat": lat, "lon": lon, "agl": agl, "roll": roll, "pitch": pitch, "yaw": yaw,
                    "gps": list(self.fc.gps), "mode": s["mode"], "size": [frame.shape[1], frame.shape[0]]}
            try:
                self.q.put_nowait((self.n, frame, meta))
                self.n += 1
                self.stats["photos"] += 1
                last_ll = (s["lat"], s["lon"])          # same reference the distance test uses
            except queue.Full:
                self.stats["dropped"] += 1

    def _worker(self):
        while not (self.stop_evt.is_set() and self.q.empty()):
            try:
                n, frame, meta = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            t0 = time.time()
            name = f"{n:05d}"
            cv2.imwrite(os.path.join(self.photos_dir, name + ".jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            dets = []
            if self.detector is not None:
                try:
                    dets = tiled_detect(self.detector, frame, self.det_cfg["tile"], self.det_cfg["tile_overlap"],
                                        self.det_cfg["conf"], (self.det_cfg["person_class"],))
                except Exception as e:
                    print(f"[vision] detection error: {e}", flush=True)
            h, w = frame.shape[:2]
            K = intrinsics_from_hfov(w, h, self.cam_cfg["hfov_deg"])
            att = camera_attitude(meta, self.cam_cfg)
            meta["detections"] = []
            for (x1, y1, x2, y2, conf) in dets:
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                off = pixel_to_ned_offset(u, v, K, att, meta["agl"])
                if off is None:
                    continue
                lat, lon = offset_to_latlon(meta["lat"], meta["lon"], off[0], off[1])
                c = self.clusters.add(lat, lon, conf, name)
                meta["detections"].append({"box": [x1, y1, x2, y2], "conf": conf, "lat": lat, "lon": lon, "survivor": c["id"]})
                self.raw_dets.append({"photo": name, "conf": conf, "lat": lat, "lon": lon})
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
                cv2.putText(frame, f"{conf:.2f}", (int(x1), max(20, int(y1) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            self.stats["raw_detections"] += len(meta["detections"])
            json.dump(meta, open(os.path.join(self.photos_dir, name + ".json"), "w"))
            small = cv2.resize(frame, (960, int(960 * h / w)))
            if meta["detections"]:
                cv2.imwrite(os.path.join(self.annot_dir, name + ".jpg"), small, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self._last_annotated = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            self.stats["processed"] += 1
            self.stats["last_ms"] = int((time.time() - t0) * 1000)
            self._write_outputs()

    def _track_loop(self):
        path = os.path.join(self.out, "track.csv")
        with open(path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t_wall", "lat", "lon", "agl", "mode", "batt_pct", "sats"])
            while not self.stop_evt.is_set():
                s = self.fc.snapshot()
                if s["lat"] is not None:
                    wr.writerow([f"{time.time():.2f}", f"{s['lat']:.7f}", f"{s['lon']:.7f}", f"{s['agl']:.1f}",
                                 s["mode"], s["batt_pct"], s["sats"]])
                    f.flush()
                    self.track.append((s["lat"], s["lon"]))
                    if len(self.track) > 3000:
                        self.track = self.track[-3000:]
                time.sleep(0.5)

    # ----- outputs
    def _write_outputs(self):
        surv = self.clusters.snapshot()
        fc_ = {"type": "FeatureCollection", "features": [
            {"type": "Feature", "properties": {"photo": d["photo"], "conf": d["conf"]},
             "geometry": {"type": "Point", "coordinates": [d["lon"], d["lat"]]}} for d in list(self.raw_dets)]}
        for name, obj in (("detections.geojson", fc_), ("survivors.json", surv)):
            tmp = os.path.join(self.out, name + ".tmp")
            json.dump(obj, open(tmp, "w"))
            os.replace(tmp, os.path.join(self.out, name))

    # ----- for the live screen
    def live_jpeg(self):
        frame, _ = self.grab.latest()
        if frame is None:
            return None
        small = cv2.resize(frame, (640, int(640 * frame.shape[0] / frame.shape[1])))
        return cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()

    def last_detection_jpeg(self):
        return self._last_annotated

    def ui_state(self):
        self.stats["camera_age_s"] = round(self.grab.age(), 1)
        return {"drone": self.fc.snapshot(), "stats": self.stats, "survivors": self.clusters.snapshot(),
                "track": self.track[-600:], "trigger_m": self.trigger_m}
