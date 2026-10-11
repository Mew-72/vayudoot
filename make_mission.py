#!/usr/bin/env python3
"""Plan the survey offline: geofence + lawnmower path + files you can inspect before flying.

usage:  python make_mission.py demo_config.json [--home LAT,LON] [--out out]

Outputs (in --out):
  mission.json      used by main_demo.py (fence, path, camera geometry)
  mission.waypoints backup: load in Mission Planner / QGC (QGC WPL 110 format)
  fence.poly        backup: Mission Planner polygon (lat lon per line)
  preview.geojson   open on geojson.io to eyeball boundary / fence / path
"""
import argparse
import json
import math
import os
import sys

from shapely.geometry import LineString, Polygon

from geo import LocalFrame, dist_m, point_in_polygon


def rot_pt(p, ang_deg, origin):
    a = math.radians(ang_deg)
    c, s = math.cos(a), math.sin(a)
    x, y = p[0] - origin[0], p[1] - origin[1]
    return (origin[0] + x * c - y * s, origin[1] + x * s + y * c)


def long_edge_angle(poly):
    rect = poly.minimum_rotated_rectangle
    xs, ys = rect.exterior.coords.xy
    best, ang = -1.0, 0.0
    for i in range(4):
        dx, dy = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
        length = math.hypot(dx, dy)
        if length > best:
            best, ang = length, math.degrees(math.atan2(dy, dx))
    return ang


def lawnmower(poly, spacing, angle_deg, reverse=False, flip0=False):
    """Back-and-forth waypoints inside poly (local metres). Returns list of (x, y)."""
    c = poly.centroid
    origin = (c.x, c.y)
    from shapely import affinity
    rot = affinity.rotate(poly, -angle_deg, origin=origin)
    minx, miny, maxx, maxy = rot.bounds
    n = max(1, math.ceil((maxy - miny) / spacing))
    step = (maxy - miny) / n
    ys = [miny + (i + 0.5) * step for i in range(n)]
    if reverse:
        ys.reverse()
    pts, flip = [], flip0
    for y in ys:
        seg = rot.intersection(LineString([(minx - 1.0, y), (maxx + 1.0, y)]))
        if seg.geom_type == "LineString":
            parts = [seg]
        else:
            parts = [g for g in getattr(seg, "geoms", []) if g.geom_type == "LineString"]
        ends = []
        for g in parts:
            if g.length < 0.5:
                continue
            a, b = sorted([g.coords[0], g.coords[-1]])      # left -> right
            ends.append((a, b))
        ends.sort(key=lambda ab: ab[0][0])
        if flip:
            ends = [(b, a) for (a, b) in ends[::-1]]
        for a, b in ends:
            pts += [a, b]
        flip = not flip
    return [rot_pt(p, angle_deg, origin) for p in pts]


def path_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def write_waypoints(path, home, wps, alt, speed):
    rows = []
    rows.append((0, 1, 0, 16, 0, 0, 0, 0, home[0], home[1], 0.0))              # home placeholder
    rows.append((1, 0, 3, 178, 1, speed, -1, 0, 0.0, 0.0, 0.0))                # change speed
    for lat, lon in wps:
        rows.append((len(rows), 0, 3, 16, 0, 0, 0, 0, lat, lon, alt))
    rows.append((len(rows), 0, 3, 20, 0, 0, 0, 0, 0.0, 0.0, 0.0))              # RTL
    with open(path, "w") as f:
        f.write("QGC WPL 110\n")
        for r in rows:
            f.write("\t".join(str(x) if not isinstance(x, float) else f"{x:.8f}" for x in r) + "\t1\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--home", default=None, help="launch point 'lat,lon' (overrides config home_latlon)")
    ap.add_argument("--out", default="out")
    args = ap.parse_args()

    cfg = json.load(open(args.config))
    corners = cfg["boundary_latlon"]
    if len(corners) < 3:
        sys.exit("boundary_latlon is empty. Put the corners of your SAFE flying area (lat, lon) in the config.")

    home = None
    if args.home:
        home = tuple(float(v) for v in args.home.split(","))
    elif cfg.get("home_latlon"):
        home = tuple(cfg["home_latlon"])

    lat0 = sum(c[0] for c in corners) / len(corners)
    lon0 = sum(c[1] for c in corners) / len(corners)
    F = LocalFrame(lat0, lon0)

    boundary = Polygon([F.to_xy(la, lo) for la, lo in corners])
    if not boundary.is_valid or boundary.area < 100:
        sys.exit("Boundary polygon is invalid or tiny. Check the order of corners (go around the field once).")

    m = cfg["margins"]
    path_poly = boundary.buffer(-m["path_m"], join_style=2)
    fence_poly = boundary.buffer(-m["fence_m"], join_style=2)
    if path_poly.is_empty or path_poly.geom_type != "Polygon":
        sys.exit("Boundary is too small for the chosen margins. Reduce margins.path_m or enlarge the boundary.")
    if m["path_m"] <= m["fence_m"] + 2.0:
        print("WARNING: path margin should be at least ~2 m larger than the fence margin.")
    if boundary.convex_hull.area - boundary.area > 1e-3 * boundary.area:
        print("WARNING: boundary is not convex. RTL / strip transitions may cut across outside areas. Check preview.geojson.")

    h, v, cam, ov = cfg["altitude_agl_m"], cfg["speed_mps"], cfg["camera"], cfg["overlap"]
    swath = 2.0 * h * math.tan(math.radians(cam["hfov_deg"]) / 2.0)
    along = swath * cam["height"] / cam["width"]
    spacing = swath * (1.0 - ov["side"])
    trigger = along * (1.0 - ov["forward"])

    angle = cfg.get("sweep_angle_deg")
    if angle is None:
        angle = long_edge_angle(path_poly)

    best = None
    for reverse in (False, True):
        for flip0 in (False, True):
            pts = lawnmower(path_poly, spacing, angle, reverse, flip0)
            if not pts:
                continue
            d = 0.0
            if home:
                hx, hy = F.to_xy(*home)
                d = math.hypot(pts[0][0] - hx, pts[0][1] - hy)
            if best is None or d < best[0]:
                best = (d, pts)
    if best is None:
        sys.exit("Could not generate a path.")
    pts_xy = best[1]
    path_ll = [F.to_ll(x, y) for x, y in pts_xy]

    fence_simpl = fence_poly.simplify(0.05)
    fence_ll = [F.to_ll(x, y) for x, y in list(fence_simpl.exterior.coords)[:-1]]

    if home and not point_in_polygon(home[0], home[1], fence_ll):
        print("WARNING: home_latlon is OUTSIDE the fence polygon. The drone will refuse to arm. Move the launch point inside.")

    L = path_length(pts_xy)
    turns = max(0, len(pts_xy) - 2)
    est_time = 15 + L / v + turns * 3 + 25
    est_photos = int(L / trigger) + 1

    os.makedirs(args.out, exist_ok=True)
    mission = {
        "name": cfg.get("name", ""),
        "origin": [lat0, lon0],
        "boundary": corners,
        "fence": fence_ll,
        "path": path_ll,
        "altitude_agl_m": h,
        "speed_mps": v,
        "swath_m": swath,
        "strip_spacing_m": spacing,
        "trigger_m": trigger,
        "sweep_angle_deg": angle,
        "est_path_m": L,
        "est_time_s": est_time,
        "est_photos": est_photos,
        "home_hint": list(home) if home else None,
    }
    json.dump(mission, open(os.path.join(args.out, "mission.json"), "w"), indent=2)

    home_for_file = home if home else path_ll[0]
    write_waypoints(os.path.join(args.out, "mission.waypoints"), home_for_file, path_ll, h, v)
    with open(os.path.join(args.out, "fence.poly"), "w") as f:
        f.write("#saved by make_mission.py\n")
        for la, lo in fence_ll:
            f.write(f"{la:.8f} {lo:.8f}\n")

    def ring(ll):
        return [[lo, la] for la, lo in ll] + [[ll[0][1], ll[0][0]]]
    gj = {"type": "FeatureCollection", "features": [
        {"type": "Feature", "properties": {"name": "boundary"}, "geometry": {"type": "Polygon", "coordinates": [ring(corners)]}},
        {"type": "Feature", "properties": {"name": "fence"}, "geometry": {"type": "Polygon", "coordinates": [ring(fence_ll)]}},
        {"type": "Feature", "properties": {"name": "path"}, "geometry": {"type": "LineString", "coordinates": [[lo, la] for la, lo in path_ll]}},
    ]}
    json.dump(gj, open(os.path.join(args.out, "preview.geojson"), "w"))

    print(f"Altitude {h} m, speed {v} m/s")
    print(f"Swath {swath:.1f} m | strip spacing {spacing:.1f} m | photo every {trigger:.1f} m")
    print(f"Waypoints {len(path_ll)} | path {L:.0f} m | est. {est_time/60:.1f} min | ~{est_photos} photos")
    print(f"Fence vertices {len(fence_ll)} | sweep angle {angle:.1f} deg")
    print("Files written to", args.out)


if __name__ == "__main__":
    main()
