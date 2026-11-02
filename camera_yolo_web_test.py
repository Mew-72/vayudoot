"""Standalone camera and YOLO test dashboard.

This script has no MAVLink, flight-controller, mission, or SITL dependency. It opens a
camera (or video/RTSP source), optionally runs YOLO person detection, and provides a
small web UI at http://localhost:8000.

Examples:
    python camera_yolo_web_test.py
    python camera_yolo_web_test.py --camera 1 --weights yolov8n.pt
    python camera_yolo_web_test.py --camera rtsp://camera.example/live --no-yolo
"""
import argparse
import os
import threading
import time

import cv2
from flask import Flask, Response, jsonify


PAGE = """<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">
  <title>Camera + YOLO Test</title>
  <style>
    body { margin: 0; background: #101318; color: #eef2f7; font-family: system-ui, sans-serif; }
    main { max-width: 1100px; margin: auto; padding: 20px; }
    h1 { margin: 0 0 6px; } p { color: #aab5c3; margin-top: 0; }
    .grid { display: grid; grid-template-columns: minmax(0, 3fr) minmax(220px, 1fr); gap: 16px; }
    .panel { background: #1b212b; border-radius: 10px; padding: 14px; }
    img { display: block; width: 100%; background: #000; border-radius: 6px; }
    dl { margin: 0; } dt { color: #9ca9ba; margin-top: 12px; } dd { margin: 3px 0 0; font-weight: 600; }
    .ok { color: #53d38c; } .bad { color: #ff7777; }
  </style>
</head>
<body><main>
  <h1>Camera + YOLO Test</h1>
  <p>Standalone test — no SITL or drone connection required.</p>
  <section class=\"grid\">
    <div class=\"panel\"><img src=\"/video\" alt=\"Live camera feed\"></div>
    <aside class=\"panel\"><h2>Status</h2><dl id=\"status\"></dl></aside>
  </section>
</main>
<script>
  const status = document.getElementById('status');
  async function refresh() {
    try {
      const s = await (await fetch('/state.json')).json();
      status.innerHTML = Object.entries(s).map(([key, value]) =>
        `<dt>${key.replaceAll('_', ' ')}</dt><dd class="${value === false ? 'bad' : value === true ? 'ok' : ''}">${value}</dd>`
      ).join('');
    } catch (_) { status.innerHTML = '<dt>Connection</dt><dd class="bad">Waiting for server</dd>'; }
  }
  refresh(); setInterval(refresh, 1000);
</script></body></html>"""


def parse_camera(value):
    """Treat numeric values as camera indexes; retain paths and URLs as strings."""
    try:
        return int(value)
    except ValueError:
        return value


class CameraYoloTest:
    def __init__(self, camera, width, height, fps, weights, confidence, imgsz, detect):
        self.camera_source = camera
        self.width = width
        self.height = height
        self.target_fps = fps
        self.confidence = confidence
        self.imgsz = imgsz
        self.detect_enabled = detect
        self.model = None
        self.model_error = None
        self._frame = None
        self._frame_time = 0.0
        self._frame_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.frames = 0
        self.detections = 0
        self.inference_ms = 0

        self.cap = self._open_camera(camera)
        self.camera_open = self.cap.isOpened()
        if self.camera_open:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if detect:
            try:
                from ultralytics import YOLO
                self.model = YOLO(weights, task="detect")
            except Exception as exc:
                self.model_error = str(exc)
                print(f"[camera-test] YOLO disabled: {exc}", flush=True)

    @staticmethod
    def _open_camera(camera):
        if isinstance(camera, int):
            backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_V4L2
            cap = cv2.VideoCapture(camera, backend)
            if not cap.isOpened():
                cap.release()
                cap = cv2.VideoCapture(camera)
            return cap
        return cv2.VideoCapture(camera)

    def start(self):
        if self.camera_open:
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        self.cap.release()

    def _capture_loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            self.camera_open = True
            annotated = self._annotate(frame)
            with self._frame_lock:
                self._frame = annotated
                self._frame_time = time.monotonic()
                self.frames += 1

    def _annotate(self, frame):
        count = 0
        self.inference_ms = 0
        if self.model is not None:
            try:
                started = time.perf_counter()
                # COCO class 0 is person; remove classes=[0] to show every model class.
                result = self.model.predict(
                    frame, imgsz=self.imgsz, conf=self.confidence, classes=[0], verbose=False
                )[0]
                self.inference_ms = round((time.perf_counter() - started) * 1000)
                for box in result.boxes:
                    x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                    score = float(box.conf[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
                    cv2.putText(frame, f"person {score:.2f}", (x1, max(y1 - 8, 22)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 0), 2)
                    count += 1
            except Exception as exc:
                self.model_error = str(exc)
                self.model = None
                print(f"[camera-test] YOLO inference disabled: {exc}", flush=True)

        self.detections = count
        label = "YOLO OFF" if self.model is None else f"People: {count} | {self.inference_ms} ms"
        color = (0, 90, 255) if self.model is None else (0, 220, 0)
        cv2.putText(frame, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        return frame

    def latest_jpeg(self):
        with self._frame_lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return encoded.tobytes() if ok else None

    def state(self):
        with self._frame_lock:
            age = None if self._frame is None else round(time.monotonic() - self._frame_time, 2)
        return {
            "camera_open": self.camera_open,
            "source": self.camera_source,
            "resolution": f"{self.width} x {self.height}",
            "frames_captured": self.frames,
            "frame_age_seconds": age if age is not None else "waiting",
            "yolo_enabled": self.model is not None,
            "people_detected": self.detections,
            "inference_ms": self.inference_ms if self.model is not None else "n/a",
            "yolo_error": self.model_error or "none",
        }


def create_app(test):
    app = Flask(__name__)

    @app.get("/")
    def index():
        return PAGE

    @app.get("/state.json")
    def state():
        return jsonify(test.state())

    @app.get("/video")
    def video():
        def frames():
            while not test._stop.is_set():
                image = test.latest_jpeg()
                if image is not None:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + image + b"\r\n"
                time.sleep(1 / max(test.target_fps, 1))
        return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def main():
    parser = argparse.ArgumentParser(description="Standalone camera and YOLO web test (no SITL needed).")
    parser.add_argument("--camera", default="0", type=parse_camera, help="Camera index, video file, or RTSP URL (default: 0)")
    parser.add_argument("--weights", default="yolov8n.pt", help="Ultralytics YOLO weights path or model name")
    parser.add_argument("--no-yolo", action="store_true", help="Show the camera only; do not load YOLO")
    parser.add_argument("--width", default=1280, type=int)
    parser.add_argument("--height", default=720, type=int)
    parser.add_argument("--fps", default=15, type=int)
    parser.add_argument("--confidence", default=0.35, type=float)
    parser.add_argument("--imgsz", default=640, type=int)
    parser.add_argument("--host", default="0.0.0.0", help="Web server bind address")
    parser.add_argument("--port", default=8000, type=int)
    args = parser.parse_args()

    test = CameraYoloTest(args.camera, args.width, args.height, args.fps, args.weights,
                          args.confidence, args.imgsz, not args.no_yolo)
    test.start()
    print(f"[camera-test] Open http://localhost:{args.port}", flush=True)
    if not test.camera_open:
        print(f"[camera-test] Could not open camera source: {args.camera!r}", flush=True)

    try:
        create_app(test).run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    finally:
        test.stop()


if __name__ == "__main__":
    main()
