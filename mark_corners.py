#!/usr/bin/env python3
"""Record the field corners with the DRONE'S OWN GPS (the same GPS the geofence will use).

Carry the drone (powered, GPS locked, props OFF) to each corner of your SAFE flying area, in order around the field,
and press Enter. Type q + Enter when done. Paste the printed list into demo_config.json -> boundary_latlon.
usage: python mark_corners.py --master /dev/serial0
"""
import argparse
import json
import time

from fc import FC


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--master", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()
    fc = FC(args.master, args.baud)
    time.sleep(2)
    pts = []
    print("Stand still at a corner and press Enter to record it. q + Enter to finish.")
    while True:
        s = fc.snapshot()
        print(f"  now: fix={s['fix']} sats={s['sats']} hdop={s['hdop']:.1f}  lat={s['lat']} lon={s['lon']}")
        cmd = input(f"corner {len(pts) + 1} > ").strip().lower()
        if cmd == "q":
            break
        if s["lat"] is None or s["fix"] < 3:
            print("  no 3D fix yet, wait and try again")
            continue
        # average ~3 s of fixes to reduce noise
        samples = []
        for _ in range(6):
            time.sleep(0.5)
            snap = fc.snapshot()
            samples.append((snap["lat"], snap["lon"]))
        lat = sum(p[0] for p in samples) / len(samples)
        lon = sum(p[1] for p in samples) / len(samples)
        pts.append([round(lat, 7), round(lon, 7)])
        print(f"  recorded corner {len(pts)}: {pts[-1]}")
    print("\n\"boundary_latlon\":", json.dumps(pts))


if __name__ == "__main__":
    main()
