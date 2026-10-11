"""Fake ArduCopter, just enough MAVLink to rehearse main_demo.py on a laptop WITHOUT SITL.
Run:  python fake_autopilot.py   then   python main_demo.py --master udpin:127.0.0.1:14560 ...
NOT a flight simulator: positions are a straight line east. Use real SITL for the proper rehearsal."""
import os, sys, time, math, threading
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
mv = mavutil.mavlink
m = mavutil.mavlink_connection("udpout:127.0.0.1:14560", source_system=1, source_component=1)
st = dict(mode=0, armed=False, alt=0.0, target_alt=0.0, lat=-35.363261, lon=149.165230, auto_t=None)
store = {0: [], 1: []}
pending = {}
def hb():
    base = 1 | (128 if st["armed"] else 0)
    m.mav.heartbeat_send(2, 3, base, st["mode"], 4)
def streams():
    t0 = time.time()
    while True:
        dt = 0.1
        if st["armed"] and st["mode"] == 4 and st["target_alt"] > st["alt"]:
            st["alt"] = min(st["target_alt"], st["alt"] + 2.0 * dt)
        if st["armed"] and st["mode"] == 3:           # AUTO: fly east at 3 m/s for 8 s then RTL
            if st["auto_t"] is None: st["auto_t"] = time.time()
            st["lon"] += 3.0 * dt / (111320 * math.cos(math.radians(st["lat"])))
            if os.environ.get("OVERRIDE") and time.time() - st["auto_t"] > 3: st["mode"] = 5; st["ov_t"] = time.time()
            elif time.time() - st["auto_t"] > 8: st["mode"] = 6; st["rtl_t"] = time.time()
        if st["armed"] and st["mode"] == 5 and time.time() - st.get("ov_t", 0) > 6: st["armed"] = False
        if st["armed"] and st["mode"] == 6:
            st["alt"] = max(0, st["alt"] - 3.0 * dt)
            if st["alt"] <= 0: st["armed"] = False; st["mode"] = 6
        ms = int((time.time() - t0) * 1000)
        m.mav.attitude_send(ms, 0.0, 0.0, math.radians(90), 0, 0, 0)
        m.mav.global_position_int_send(ms, int(st["lat"] * 1e7), int(st["lon"] * 1e7), int(st["alt"] * 1000), int(st["alt"] * 1000), 0, 0, 0, 9000)
        m.mav.gps_raw_int_send(ms * 1000, 3, int(st["lat"] * 1e7), int(st["lon"] * 1e7), 0, 80, 100, 0, 0, 12)
        m.mav.sys_status_send(0, 0, 0, 500, 12000, -1, 85, 0, 0, 0, 0, 0, 0)
        time.sleep(dt)
def hbloop():
    while True:
        hb(); time.sleep(1)
threading.Thread(target=streams, daemon=True).start()
threading.Thread(target=hbloop, daemon=True).start()
while True:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None: continue
    k = msg.get_type()
    if k == "COMMAND_LONG":
        c = msg.command; ok = 0; print("FAKE cmd", c, "p2", msg.param2, "mode", st["mode"], flush=True)
        if c == 176: st["mode"] = int(msg.param2)
        elif c == 400: st["armed"] = msg.param1 == 1
        elif c == 22: st["target_alt"] = msg.param7
        m.mav.command_ack_send(c, ok)
    elif k == "PARAM_SET":
        m.mav.param_value_send(msg.param_id.encode(), msg.param_value, 9, 1, 0)
    elif k == "MISSION_COUNT":
        t = msg.mission_type; pending[t] = (msg.count, []); 
        if msg.count: m.mav.mission_request_int_send(msg.get_srcSystem(), msg.get_srcComponent(), 0, t)
    elif k == "MISSION_ITEM_INT":
        t = msg.mission_type; cnt, items = pending[t]; items.append(msg)
        if len(items) < cnt: m.mav.mission_request_int_send(msg.get_srcSystem(), msg.get_srcComponent(), len(items), t)
        else:
            store[t] = items; m.mav.mission_ack_send(msg.get_srcSystem(), msg.get_srcComponent(), 0, t)
            print("FAKE: stored", len(items), "items of type", t, flush=True)
    elif k == "MISSION_CLEAR_ALL":
        store[msg.mission_type] = []; m.mav.mission_ack_send(msg.get_srcSystem(), msg.get_srcComponent(), 0, msg.mission_type)
    elif k == "MISSION_REQUEST_LIST":
        m.mav.mission_count_send(msg.get_srcSystem(), msg.get_srcComponent(), len(store[msg.mission_type]), msg.mission_type)
