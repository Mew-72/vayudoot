# Scout drone demo pack

Autonomous survey of a field: plan -> geofence + mission upload -> arm -> take off -> AUTO lawnmower -> RTL -> land,
while the Pi takes a photo every few metres, finds people (YOLO), geotags them and shows everything on a live screen.

## Files
| File | What it does |
|---|---|
| demo_config.json | **Edit this.** Field corners, altitude, speed, camera, margins, safety limits |
| sitl_test_field.json | Ready-made config around the default SITL home, for rehearsals on a laptop |
| mark_corners.py | Record the field corners with the drone's own GPS (carry it to each corner) |
| make_mission.py | Plans the survey offline -> out/mission.json, mission.waypoints, fence.poly, preview.geojson |
| main_demo.py | The autonomous flight program (run on the Pi) |
| fc.py / mission_io.py | MAVLink link, fence + mission upload |
| vision.py / geotag.py / geo.py | Camera, photo logger, tiled detection, geotagging, duplicate merging |
| webui.py / ui_assets.py | Live screen on port 8000 (works offline, no internet needed) |
| report.py | Makes report.html after the flight |
| fake_autopilot.py | Protocol-only stand-in for rehearsals (not a simulator) |

## Quick start
```bash
python3 -m venv --system-site-packages ~/rs-env && source ~/rs-env/bin/activate
pip install -r requirements.txt

python make_mission.py demo_config.json --home LAT,LON      # check out/preview.geojson on geojson.io
python main_demo.py --dry-run --master /dev/serial0         # hand-carry test, never arms
python main_demo.py --master /dev/serial0                   # real flight: type FLY to start, Ctrl+C = abort
python report.py out/mission.json session_YYYYMMDD_HHMMSS   # report.html
```
Export the detector once (any machine with internet): `yolo export model=yolov8n.pt format=ncnn imgsz=640`
and copy the `yolov8n_ncnn_model` folder next to the scripts.

**Read the guide before flying. This code was tested against a protocol stand-in, not against a real ArduPilot. Run it in SITL first.**
