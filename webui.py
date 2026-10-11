"""Tiny live screen for the audience (map + camera + survivors). Open http://<pi-address>:8000 on a laptop."""
import json
import logging
import threading
import time

from flask import Flask, Response, jsonify

from ui_assets import LIVE_HTML


def start(survey, mission_path, port=8000):
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    app = Flask(__name__)

    @app.route("/")
    def index():
        return LIVE_HTML

    @app.route("/state.json")
    def state():
        return jsonify(survey.ui_state())

    @app.route("/mission.json")
    def mission():
        return Response(open(mission_path).read(), mimetype="application/json")

    @app.route("/last.jpg")
    def last():
        jpg = survey.last_detection_jpeg()
        if jpg is None:
            return Response(status=204)
        return Response(jpg, mimetype="image/jpeg")

    @app.route("/video")
    def video():
        def gen():
            while True:
                jpg = survey.live_jpeg()
                if jpg:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                time.sleep(0.25)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=port, threaded=True, use_reloader=False),
                     daemon=True).start()
    return app
