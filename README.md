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

## Raspberry Pi quick start
```bash
python3 -m venv --system-site-packages ~/rs-env && source ~/rs-env/bin/activate
pip install -r requirements.txt

python make_mission.py demo_config.json --home LAT,LON      # check out/preview.geojson on geojson.io
python main_demo.py --dry-run --master /dev/serial0         # hand-carry test, never arms
python main_demo.py --master /dev/serial0                   # real flight: type FLY to start, Ctrl+C = abort
python report.py out/mission.json session_YYYYMMDD_HHMMSS   # writes report.html
```

Export the detector once on a machine with internet, then place the generated `yolov8n_ncnn_model` directory next to the scripts:

```bash
pip install ultralytics ncnn
yolo export model=yolov8n.pt format=ncnn imgsz=640
```

## Windows + WSL ArduPilot SITL rehearsal

Start ArduCopter SITL in WSL. In the MAVProxy console, send telemetry to the Windows host through the WSL gateway (replace the address if your WSL gateway differs):

```text
output add [IP ADDRESS]:14550
output
```

On Windows PowerShell, create the mission and run the dry-run. `0.0.0.0` is intentional: it receives UDP packets forwarded from WSL; `127.0.0.1` may not receive them.

```powershell
python make_mission.py sitl_test_field.json --out out
python main_demo.py --config sitl_test_field.json --mission out/mission.json --master udpin:0.0.0.0:14550 --dry-run
```

Open the live dashboard locally:

```text
http://localhost:8000
http://localhost:8000/state.json
```

Press `Ctrl+C` once to finish the dry run, then make the offline report:

```powershell
python report.py out/mission.json session_YYYYMMDD_HHMMSS
```

For a simulated location other than the default SITL site, start SITL with matching coordinates:

```bash
sim_vehicle.py -v ArduCopter --custom-location=LATITUDE,LONGITUDE,ELEVATION_METRES,HEADING_DEGREES
```

The mission configuration's `home_latlon` and `boundary_latlon` must be at that same location. Do not use a real-world field configuration with SITL still running at its default location.

## Pi and GCS laptop deployment

Run `main_demo.py` on the Pi, where its camera and serial MAVLink connection to the flight controller are available. Put the Pi and GCS laptop on the same Wi-Fi or Ethernet network. Find the Pi address:

```bash
hostname -I
```

From the GCS laptop, open:

```text
http://PI_IP_ADDRESS:8000
```

Use Mission Planner or QGroundControl on an independent flight-controller telemetry link for vehicle monitoring. The Pi dashboard provides camera, detection, and survivor data; generated session folders stay on the Pi and can be copied afterward:

```bash
scp -r pi@PI_IP_ADDRESS:~/rescue_demo_pack/session_YYYYMMDD_HHMMSS .
```

**Read the guide before flying. This code was tested against a protocol stand-in, not against a real ArduPilot. Run it in SITL first. `--dry-run` does not arm or take off, but it does upload the mission/fence and change flight-controller parameters.**
