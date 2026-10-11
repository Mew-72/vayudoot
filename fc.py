"""One MAVLink connection shared by everything (a serial port can only be opened once).

A single reader thread dispatches all messages; other code asks for state or waits for replies.
"""
import math
import queue
import threading
import time
import collections

import os
os.environ.setdefault("MAVLINK20", "1")   # MAVLink 2 is required for fence / mission_type
from pymavlink import mavutil

COPTER_MODES = {"STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
                "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11, "SPORT": 13, "FLIP": 14,
                "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18, "GUIDED_NOGPS": 20,
                "SMART_RTL": 21}
MODE_NAMES = {v: k for k, v in COPTER_MODES.items()}
EVENT_TYPES = {"MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK", "MISSION_COUNT",
               "COMMAND_ACK", "PARAM_VALUE", "MISSION_ITEM_REACHED"}


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class FC:
    def __init__(self, conn="/dev/serial0", baud=921600):
        self.m = mavutil.mavlink_connection(conn, baud=baud, source_system=255, source_component=191)
        self.lock = threading.Lock()
        self.events = queue.Queue()
        self.att = collections.deque(maxlen=900)    # (t, roll, pitch, yaw)
        self.pos = collections.deque(maxlen=400)    # (t, lat, lon, rel_alt_m)
        self.gps = (0, 0, 99.0)                     # fix_type, satellites, hdop
        self.batt_pct, self.batt_v = -1, 0.0
        self.armed, self.custom_mode, self.last_hb = False, None, None
        self.last_mission_reached = None
        self._stop = False

        while True:   # find the autopilot's heartbeat (ignore gimbal, GCS, ...)
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=15)
            if hb is None:
                raise TimeoutError("No heartbeat from the flight controller. Check wiring, port and baud rate.")
            if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.sysid, self.compid = hb.get_srcSystem(), hb.get_srcComponent()
                self.custom_mode = hb.custom_mode
                self.armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.last_hb = time.monotonic()
                break
        self.m.target_system, self.m.target_component = self.sysid, self.compid

        ids = mavutil.mavlink
        for msg_id, hz in [(ids.MAVLINK_MSG_ID_ATTITUDE, 30), (ids.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
                           (ids.MAVLINK_MSG_ID_GPS_RAW_INT, 2), (ids.MAVLINK_MSG_ID_SYS_STATUS, 2)]:
            self.command_long(ids.MAV_CMD_SET_MESSAGE_INTERVAL, [msg_id, int(1e6 / hz), 0, 0, 0, 0, 0], timeout=1.0)
        threading.Thread(target=self._loop, daemon=True).start()

    # ------------------------------------------------------------------ reader
    def _loop(self):
        next_hb = 0.0
        while not self._stop:
            now = time.monotonic()
            if now >= next_hb:
                with self.lock:
                    self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                              mavutil.mavlink.MAV_STATE_ACTIVE)
                next_hb = now + 1.0
            msg = self.m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            k = msg.get_type()
            if k == "BAD_DATA":
                continue
            t = time.monotonic()
            if k == "HEARTBEAT":
                if msg.get_srcSystem() == self.sysid and msg.get_srcComponent() == self.compid:
                    self.custom_mode = msg.custom_mode
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.last_hb = t
            elif k == "ATTITUDE":
                self.att.append((t, msg.roll, msg.pitch, msg.yaw))
            elif k == "GLOBAL_POSITION_INT":
                self.pos.append((t, msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0))
            elif k == "GPS_RAW_INT":
                hdop = 99.0 if msg.eph in (0, 65535) else msg.eph / 100.0
                self.gps = (msg.fix_type, msg.satellites_visible, hdop)
            elif k == "SYS_STATUS":
                self.batt_pct, self.batt_v = msg.battery_remaining, msg.voltage_battery / 1000.0
            elif k == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else bytes(msg.text).decode(errors="ignore")
                print(f"[FC] {text}", flush=True)
            elif k == "MISSION_ITEM_REACHED":
                self.last_mission_reached = msg.seq
            if k in EVENT_TYPES:
                self.events.put(msg)

    def close(self):
        self._stop = True

    # ------------------------------------------------------------------ helpers
    def clear_events(self):
        while not self.events.empty():
            try:
                self.events.get_nowait()
            except queue.Empty:
                break

    def wait_event(self, types, timeout, pred=None):
        end = time.monotonic() + timeout
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return None
            try:
                msg = self.events.get(timeout=remaining)
            except queue.Empty:
                return None
            if msg.get_type() in types and (pred is None or pred(msg)):
                return msg

    def command_long(self, cmd, params=(0,) * 7, timeout=3.0):
        p = list(params) + [0] * (7 - len(params))
        self.clear_events()
        with self.lock:
            self.m.mav.command_long_send(self.sysid, self.compid, cmd, 0, *p)
        ack = self.wait_event({"COMMAND_ACK"}, timeout, lambda a: a.command == cmd)
        return ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

    def set_param(self, name, value, timeout=3.0):
        self.clear_events()
        with self.lock:
            self.m.mav.param_set_send(self.sysid, self.compid, name.encode("ascii"), float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        def match(p):
            pid = p.param_id if isinstance(p.param_id, str) else bytes(p.param_id).decode(errors="ignore")
            return pid.rstrip("\x00") == name
        msg = self.wait_event({"PARAM_VALUE"}, timeout, match)
        return msg is not None and abs(msg.param_value - float(value)) < 1e-3

    def get_param(self, name, timeout=3.0):
        self.clear_events()
        with self.lock:
            self.m.mav.param_request_read_send(self.sysid, self.compid, name.encode("ascii"), -1)
        def match(p):
            pid = p.param_id if isinstance(p.param_id, str) else bytes(p.param_id).decode(errors="ignore")
            return pid.rstrip("\x00") == name
        msg = self.wait_event({"PARAM_VALUE"}, timeout, match)
        return None if msg is None else msg.param_value

    def mode_name(self):
        return MODE_NAMES.get(self.custom_mode, str(self.custom_mode))

    def set_mode(self, name, timeout=5.0):
        mid = COPTER_MODES[name]
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                              [mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid, 0, 0, 0, 0, 0], timeout=1.0)
            t2 = time.monotonic() + 1.0
            while time.monotonic() < t2:
                if self.custom_mode == mid:
                    return True
                time.sleep(0.05)
        return False

    def arm(self, timeout=10.0):
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0], timeout=4.0)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.armed:
                return True
            time.sleep(0.1)
        return False

    def takeoff(self, alt_m):
        return self.command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, alt_m], timeout=4.0)

    def set_mission_current(self, seq):
        with self.lock:
            self.m.mav.mission_set_current_send(self.sysid, self.compid, seq)

    # ------------------------------------------------------------------ state
    def snapshot(self):
        p = self.pos[-1] if self.pos else None
        a = self.att[-1] if self.att else None
        return {"lat": p[1] if p else None, "lon": p[2] if p else None, "agl": p[3] if p else 0.0,
                "yaw_deg": math.degrees(a[3]) if a else 0.0, "mode": self.mode_name(), "armed": self.armed,
                "batt_pct": self.batt_pct, "batt_v": round(self.batt_v, 2), "fix": self.gps[0],
                "sats": self.gps[1], "hdop": self.gps[2],
                "link_age": (time.monotonic() - self.last_hb) if self.last_hb else None}

    @staticmethod
    def _bracket(buf, t):
        items = list(buf)
        for a, b in zip(items, items[1:]):
            if a[0] <= t <= b[0]:
                return a, b, (t - a[0]) / max(b[0] - a[0], 1e-6)
        return None

    def state_at(self, t):
        """Interpolated (lat, lon, agl, roll, pitch, yaw) at monotonic time t, or None."""
        p = self._bracket(self.pos, t)
        a = self._bracket(self.att, t)
        if p is None or a is None:
            return None
        (p0, p1, wp), (a0, a1, wa) = p, a
        return (p0[1] + wp * (p1[1] - p0[1]), p0[2] + wp * (p1[2] - p0[2]), p0[3] + wp * (p1[3] - p0[3]),
                a0[1] + wa * (a1[1] - a0[1]), a0[2] + wa * (a1[2] - a0[2]), a0[3] + wa * _wrap(a1[3] - a0[3]))
