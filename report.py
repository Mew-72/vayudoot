#!/usr/bin/env python3
"""Make a single-file HTML flight report (works offline).  usage: python report.py out/mission.json session_dir"""
import csv
import glob
import json
import os
import sys

from ui_assets import DRAW_JS, REPORT_HTML


def main():
    mission_path, session = sys.argv[1], sys.argv[2]
    mission = json.load(open(mission_path))
    survivors = json.load(open(os.path.join(session, "survivors.json"))) if os.path.exists(os.path.join(session, "survivors.json")) else []
    track = []
    tp = os.path.join(session, "track.csv")
    if os.path.exists(tp):
        for r in csv.DictReader(open(tp)):
            track.append([float(r["lat"]), float(r["lon"])])
    photos = len(glob.glob(os.path.join(session, "photos", "*.jpg")))
    data = {"mission": mission, "survivors": survivors, "track": track, "photos": photos}
    html = REPORT_HTML.replace("__DRAW__", DRAW_JS).replace("__DATA__", json.dumps(data))
    out = os.path.join(session, "report.html")
    open(out, "w").write(html)
    print("Wrote", out)


if __name__ == "__main__":
    main()
