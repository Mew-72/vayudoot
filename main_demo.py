#!/usr/bin/env python3
"""Autonomous survey demo: checks -> upload fence + mission -> arm -> take off -> AUTO survey -> RTL -> land.
The operator's only inputs: type FLY to start, Ctrl+C to abort (first press = RTL, second = LAND).
A safety pilot with the RC transmitter stays ready. If the pilot changes the flight mode, this program
stops commanding and just keeps logging.

usage:
  python main_demo.py --master udpin:127.0.0.1:14550 --dry-run        # SITL / bench, never arms
  python main_demo.py --master /dev/serial0 --baud 921600              # real flight
"""
import argparse
import json
import os
import signal
import sys
import time

import fc as fcmod
import mission_io
import webui
from geo import dist_m, point_in_polygon
from vision import Survey


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


abort_count = 0


def on_sigint(sig, frame):
    global abort_count
    abort_count += 1
    log(f"!!! ABORT requested (press {abort_count}) !!!")


def preflight(fc, cfg, mis, args):
    s = cfg["safety"]
    log("Waiting for GPS lock ...")
    t_end = time.monotonic() + 90
    while time.monotonic() < t_end:
        snap = fc.snapshot()
        if snap["lat"] is not None and snap["fix"] >= 3 and snap["sats"] >= s["min_gps_sats"] and snap["hdop"] <= s["max_hdop"]:
            break
        time.sleep(1)
    snap = fc.snapshot()
    ok = True
    if snap["lat"] is None or snap["fix"] < 3:
        log("FAIL: no 3D GPS fix"); ok = False
    if snap["sats"] < s["min_gps_sats"]:
        log(f"FAIL: only {snap['sats']} satellites (need {s['min_gps_sats']})"); ok = False
    if snap["hdop"] > s["max_hdop"]:
        log(f"FAIL: HDOP {snap['hdop']:.1f} (max {s['max_hdop']})"); ok = False
    if snap["armed"]:
        log("FAIL: vehicle is already armed"); ok = False
    if snap["lat"] is not None and not point_in_polygon(snap["lat"], snap["lon"], mis["fence"]):
        log("FAIL: the drone is OUTSIDE the geofence polygon. Move it to the launch spot inside the fence."); ok = False
    if snap["lat"] is not None:
        d0 = dist_m(snap["lat"], snap["lon"], mis["path"][0][0], mis["path"][0][1])
        log(f"Distance from drone to first waypoint: {d0:.0f} m")
    if snap["batt_pct"] < 0:
        if args.allow_unknown_battery:
            log("WARNING: battery percentage unknown (ignored by --allow-unknown-battery)")
        else:
            log("FAIL: battery percentage unknown. Configure the battery monitor, or use --allow-unknown-battery"); ok = False
    elif snap["batt_pct"] < s["min_batt_pct"]:
        log(f"FAIL: battery {snap['batt_pct']}% (min {s['min_batt_pct']}%)"); ok = False
    return ok


def set_params(fc, cfg, mis):
    s = cfg["safety"]
    # wanted = {
    #     "FENCE_ENABLE": 1,
    #     "FENCE_TYPE": 5,                 # 1 = max altitude, 4 = polygon
    #     "FENCE_ACTION": 1,               # RTL (or land)
    #     "FENCE_ALT_MAX": cfg["fence_alt_max_m"],
    #     "FENCE_MARGIN": 2,
    #     "WP_SPD": mis["speed_mps"],                  # m/s
    #     "RTL_ALT": s["rtl_alt_m"] * 100.0,           # cm
    #     "RTL_ALT_FINAL": 0,
    # }

    wanted = {
        "FENCE_ENABLE": 1,
        "FENCE_TYPE": 5,
        "FENCE_ACTION": 1,
        "FENCE_ALT_MAX": cfg["fence_alt_max_m"],
        "FENCE_MARGIN": 2,
        "WP_SPD": mis["speed_mps"],              # m/s
        "RTL_ALT_M": s["rtl_alt_m"],             # m
        "RTL_ALT_FINAL_M": 0.0,                  # m
    }


    for k, v in wanted.items():
        if not fc.set_param(k, v):
            log(f"FAIL: could not set {k}={v}")
            return False
        log(f"param {k} = {v}")
    return True


def monitor(fc, cfg, phase, t_limit):
    """Returns 'done' when disarmed, or 'override' if someone else changed the mode."""
    global abort_count
    s = cfg["safety"]
    handled = 0
    sent_rtl = False
    last_print, last_seq = 0.0, None
    t0 = time.monotonic()
    while True:
        time.sleep(0.3)
        snap = fc.snapshot()
        if not snap["armed"]:
            return "done"
        mode = snap["mode"]
        if mode not in ("AUTO", "RTL", "LAND", "SMART_RTL"):
            log(f"MODE is now {mode}: pilot override or failsafe. This program stops commanding. Logging continues.")
            return "override"
        if abort_count > handled:
            handled = abort_count
            target = "RTL" if handled == 1 else "LAND"
            log(f"Abort -> {target}")
            fc.set_mode(target)
            sent_rtl = True
        if not sent_rtl and mode == "AUTO":
            if time.monotonic() - t0 > t_limit:
                log("Time limit reached -> RTL"); fc.set_mode("RTL"); sent_rtl = True
            elif 0 <= snap["batt_pct"] < s["abort_batt_pct"]:
                log(f"Battery {snap['batt_pct']}% -> RTL"); fc.set_mode("RTL"); sent_rtl = True
        if fc.last_mission_reached != last_seq:
            last_seq = fc.last_mission_reached
            log(f"Reached mission item {last_seq}")
        if time.monotonic() - last_print > 5:
            last_print = time.monotonic()
            log(f"[{phase}] mode={mode} alt={snap['agl']:.1f} m batt={snap['batt_pct']}% sats={snap['sats']}")


def fly(fc, cfg, mis):
    h = mis["altitude_agl_m"]
    log("Mode GUIDED, arming ...")
    if not fc.set_mode("GUIDED"):
        log("FAIL: could not enter GUIDED"); return False
    if not fc.arm():
        log("FAIL: arming refused (see [FC] PreArm messages above)"); return False
    if not fc.takeoff(h):
        log("FAIL: take-off command rejected -> disarming"); fc.command_long(400, [0, 21196, 0, 0, 0, 0, 0]); return False
    log(f"Taking off to {h} m ...")
    t_end = time.monotonic() + 45
    while time.monotonic() < t_end:
        time.sleep(0.3)
        snap = fc.snapshot()
        if not snap["armed"]:
            log("Disarmed during take-off"); return False
        if snap["mode"] != "GUIDED":
            log(f"Mode changed to {snap['mode']} during take-off: pilot override or failsafe. Stopping commands.")
            while fc.snapshot()["armed"]:
                time.sleep(0.5)
            return True
        if abort_count:
            log("Abort during take-off -> RTL"); fc.set_mode("RTL"); return True
        if snap["agl"] >= 0.9 * h:
            break
    else:
        log("Take-off timeout -> RTL"); fc.set_mode("RTL"); return True
    log("At altitude. Starting AUTO survey.")
    fc.set_mission_current(1)
    if not fc.set_mode("AUTO"):
        log("Could not enter AUTO -> RTL"); fc.set_mode("RTL")
    result = monitor(fc, cfg, "survey", cfg["safety"]["max_mission_s"])
    if result == "override":
        # keep waiting (without commanding) until the aircraft is on the ground
        while fc.snapshot()["armed"]:
            time.sleep(0.5)
    return True


def main():
    global abort_count
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="demo_config.json")
    ap.add_argument("--mission", default="out/mission.json")
    ap.add_argument("--master", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--session", default=None)
    ap.add_argument("--dry-run", action="store_true", help="do everything except arm and take off")
    ap.add_argument("--min-agl", type=float, default=4.0, help="photos are only taken above this height")
    ap.add_argument("--allow-unknown-battery", action="store_true")
    ap.add_argument("--no-web", action="store_true")
    args = ap.parse_args()

    cfg = json.load(open(args.config))
    mis = json.load(open(args.mission))
    session = args.session or time.strftime("session_%Y%m%d_%H%M%S")
    os.makedirs(session, exist_ok=True)

    log(f"Connecting to {args.master} ...")
    fc = fcmod.FC(args.master, args.baud)
    log(f"Connected: system {fc.sysid}, mode {fc.mode_name()}")

    if not preflight(fc, cfg, mis, args):
        log("PRE-FLIGHT FAILED. Nothing was changed on the drone except stream rates."); sys.exit(1)
    if not set_params(fc, cfg, mis):
        sys.exit(1)
    log("Uploading geofence ...")
    if not mission_io.upload_fence(fc, mis["fence"]):
        log("FAIL: geofence upload failed"); sys.exit(1)
    snap = fc.snapshot()
    home = (snap["lat"], snap["lon"])
    log("Uploading mission ...")
    if not mission_io.upload_mission(fc, home, mis["path"], mis["altitude_agl_m"], mis["speed_mps"]):
        log("FAIL: mission upload failed"); sys.exit(1)
    log(f"Geofence ({len(mis['fence'])} vertices) and mission ({len(mis['path'])} waypoints) are on the drone.")

    survey = Survey(fc, cfg, mis, session, min_agl=(0.0 if args.dry_run else args.min_agl))
    survey.start()
    if not args.no_web:
        webui.start(survey, args.mission)
        log("Live screen on port 8000  (http://<pi-ip>:8000)")
    log(f"Session folder: {session}")

    signal.signal(signal.SIGINT, on_sigint)
    if args.dry_run:
        log("DRY RUN: not arming. Carry the drone by hand to test camera, geotag and the live screen. Ctrl+C to finish.")
        while abort_count == 0:
            time.sleep(0.5)
    else:
        print("\nEverything is ready. Safety pilot standing by? Area clear? Type FLY and press Enter to start the autonomous flight.")
        try:
            if input("> ").strip() != "FLY":
                log("Not started."); survey.stop(); return
        except EOFError:
            return
        for i in range(5, 0, -1):
            log(f"Starting in {i} ..."); time.sleep(1)
        if abort_count:
            log("Aborted before arming"); survey.stop(); return
        fly(fc, cfg, mis)
        time.sleep(3)

    log("Flight finished. Saving results ...")
    survey.stop()
    conf = [s for s in survey.clusters.snapshot() if s["confirmed"]]
    log(f"Photos: {survey.stats['photos']} | processed: {survey.stats['processed']} | confirmed survivors: {len(conf)}")
    for s in conf:
        log(f"  survivor #{s['id']}: {s['lat']:.6f}, {s['lon']:.6f}  ({s['hits']} sightings, best conf {s['conf']:.2f})")
    log(f"Make the report with:  python report.py {args.mission} {session}")


if __name__ == "__main__":
    main()
