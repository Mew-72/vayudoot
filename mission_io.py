"""Upload geofence and mission to ArduCopter over MAVLink and check the autopilot accepted them."""
import time

import os
os.environ.setdefault("MAVLINK20", "1")   # MAVLink 2 is required for fence / mission_type
from pymavlink import mavutil

MT_MISSION, MT_FENCE = 0, 1


def _clear(fc, mission_type):
    fc.clear_events()
    with fc.lock:
        fc.m.mav.mission_clear_all_send(fc.sysid, fc.compid, mission_type)
    ack = fc.wait_event({"MISSION_ACK"}, 3.0)
    return ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED


def _upload(fc, items, mission_type, timeout=15.0):
    n = len(items)
    fc.clear_events()
    with fc.lock:
        fc.m.mav.mission_count_send(fc.sysid, fc.compid, n, mission_type)
    deadline = time.monotonic() + timeout
    last_activity = time.monotonic()
    while time.monotonic() < deadline:
        ev = fc.wait_event({"MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"}, 1.0)
        if ev is None:
            if time.monotonic() - last_activity > 3.0:      # nothing happening: ask again
                with fc.lock:
                    fc.m.mav.mission_count_send(fc.sysid, fc.compid, n, mission_type)
                last_activity = time.monotonic()
            continue
        last_activity = time.monotonic()
        if ev.get_type() == "MISSION_ACK":
            return ev.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
        it = items[ev.seq]
        p = it["p"]
        with fc.lock:
            fc.m.mav.mission_item_int_send(
                fc.sysid, fc.compid, ev.seq, it["frame"], it["cmd"], 0, 1,
                p[0], p[1], p[2], p[3],
                int(round(it["lat"] * 1e7)), int(round(it["lon"] * 1e7)), float(it["alt"]), mission_type)
    return False


def _count(fc, mission_type):
    fc.clear_events()
    with fc.lock:
        fc.m.mav.mission_request_list_send(fc.sysid, fc.compid, mission_type)
    ev = fc.wait_event({"MISSION_COUNT"}, 3.0)
    return None if ev is None else ev.count


def fence_items(fence_ll):
    n = len(fence_ll)
    return [{"frame": mavutil.mavlink.MAV_FRAME_GLOBAL, "cmd": mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
             "p": [n, 0, 0, 0], "lat": la, "lon": lo, "alt": 0.0} for la, lo in fence_ll]


def mission_items(home, path_ll, alt, speed):
    wp = mavutil.mavlink
    items = [{"frame": wp.MAV_FRAME_GLOBAL, "cmd": wp.MAV_CMD_NAV_WAYPOINT, "p": [0, 0, 0, 0],
              "lat": home[0], "lon": home[1], "alt": 0.0}]                                   # seq 0 = home placeholder
    items.append({"frame": wp.MAV_FRAME_GLOBAL_RELATIVE_ALT, "cmd": wp.MAV_CMD_DO_CHANGE_SPEED,
                  "p": [1, speed, -1, 0], "lat": 0.0, "lon": 0.0, "alt": 0.0})
    for la, lo in path_ll:
        items.append({"frame": wp.MAV_FRAME_GLOBAL_RELATIVE_ALT, "cmd": wp.MAV_CMD_NAV_WAYPOINT,
                      "p": [0, 0, 0, 0], "lat": la, "lon": lo, "alt": alt})
    items.append({"frame": wp.MAV_FRAME_GLOBAL_RELATIVE_ALT, "cmd": wp.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                  "p": [0, 0, 0, 0], "lat": 0.0, "lon": 0.0, "alt": 0.0})
    return items


def upload_fence(fc, fence_ll):
    items = fence_items(fence_ll)
    _clear(fc, MT_FENCE)
    ok = _upload(fc, items, MT_FENCE)
    return ok and _count(fc, MT_FENCE) == len(items)


def upload_mission(fc, home, path_ll, alt, speed):
    items = mission_items(home, path_ll, alt, speed)
    _clear(fc, MT_MISSION)
    ok = _upload(fc, items, MT_MISSION)
    return ok and _count(fc, MT_MISSION) == len(items)
