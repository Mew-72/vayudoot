"""Offline-friendly map drawing (plain canvas, no internet needed) shared by the live screen and the report."""

DRAW_JS = r"""
function makeMap(canvas, M) {
  const k = 111320, lat0 = M.origin[0], lon0 = M.origin[1], kx = k * Math.cos(lat0 * Math.PI / 180);
  const P = (la, lo) => [(lo - lon0) * kx, (la - lat0) * k];
  const bpts = M.boundary.map(p => P(p[0], p[1]));
  const xs = bpts.map(p => p[0]), ys = bpts.map(p => p[1]);
  const minx = Math.min(...xs), maxx = Math.max(...xs), miny = Math.min(...ys), maxy = Math.max(...ys);
  return function draw(state) {
    const W = canvas.clientWidth, H = canvas.clientHeight;
    canvas.width = W; canvas.height = H;
    const ctx = canvas.getContext('2d');
    const sc = Math.min((W - 40) / (maxx - minx), (H - 40) / (maxy - miny));
    const T = (la, lo) => { const p = P(la, lo); return [20 + (p[0] - minx) * sc, H - 20 - (p[1] - miny) * sc]; };
    ctx.fillStyle = '#143d1c'; ctx.fillRect(0, 0, W, H);
    const poly = (pts, col, dash, close) => {
      ctx.beginPath(); pts.forEach((p, i) => { const q = T(p[0], p[1]); i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); });
      if (close) ctx.closePath(); ctx.setLineDash(dash); ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.stroke(); ctx.setLineDash([]);
    };
    poly(M.boundary, '#ffd400', [], true);
    poly(M.fence, '#ff4d4d', [8, 6], true);
    poly(M.path, '#4dd2ff', [4, 4], false);
    if (state.track && state.track.length > 1) poly(state.track, '#ffffff', [], false);
    (state.survivors || []).forEach(s => {
      const q = T(s.lat, s.lon);
      ctx.beginPath(); ctx.arc(q[0], q[1], 9, 0, 7);
      ctx.fillStyle = s.confirmed ? '#ff2d2d' : '#ff9f1c'; ctx.fill();
      ctx.fillStyle = '#fff'; ctx.font = '13px sans-serif'; ctx.fillText('#' + s.id + ' (' + s.hits + ')', q[0] + 12, q[1] + 4);
    });
    const d = state.drone;
    if (d && d.lat !== null && d.lat !== undefined) {
      const q = T(d.lat, d.lon), a = (d.yaw_deg || 0) * Math.PI / 180;
      ctx.save(); ctx.translate(q[0], q[1]); ctx.rotate(a);
      ctx.beginPath(); ctx.moveTo(0, -12); ctx.lineTo(8, 9); ctx.lineTo(-8, 9); ctx.closePath();
      ctx.fillStyle = '#00ff88'; ctx.fill(); ctx.restore();
    }
  };
}
"""

LIVE_HTML = """<!doctype html><html><head><meta charset="utf-8"><title>Scout GCS</title>
<style>
 body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;display:flex;height:100vh}
 #map{flex:2;min-width:0} #side{flex:1;padding:10px;overflow:auto;max-width:520px}
 h3{margin:10px 0 4px} img{width:100%;border-radius:6px;background:#000;min-height:120px}
 .row{display:flex;justify-content:space-between;border-bottom:1px solid #333;padding:2px 0}
 .big{font-size:22px;font-weight:700} .bad{color:#ff6b6b} .ok{color:#6bff95}
</style></head><body>
<canvas id="map"></canvas>
<div id="side">
 <div class="big">Scout drone &mdash; live</div>
 <div id="status"></div>
 <h3>Camera</h3><img src="/video">
 <h3>Last detection</h3><img id="last" src="/last.jpg">
 <h3>Survivors found</h3><div id="surv"></div>
 <div style="margin-top:10px;font-size:12px;color:#999">yellow = boundary, red dashed = geofence, blue dashed = planned path, white = flown track</div>
</div>
<script>
__DRAW__
let M = null, draw = null, last = {};
async function init(){ M = await (await fetch('/mission.json')).json(); draw = makeMap(document.getElementById('map'), M); tick(); setInterval(tick, 1000); }
async function tick(){
  try {
    const s = await (await fetch('/state.json')).json(); last = s; draw(s);
    const d = s.drone, st = s.stats;
    const row = (k, v, c) => `<div class="row"><span>${k}</span><span class="${c||''}">${v}</span></div>`;
    document.getElementById('status').innerHTML =
      row('Mode', d.mode + (d.armed ? ' (armed)' : ' (disarmed)')) + row('Altitude', d.agl.toFixed(1) + ' m') +
      row('Battery', d.batt_pct + ' %  /  ' + d.batt_v + ' V', d.batt_pct >= 0 && d.batt_pct < 40 ? 'bad' : 'ok') +
      row('GPS', 'fix ' + d.fix + ', ' + d.sats + ' sats, hdop ' + d.hdop.toFixed(1)) +
      row('Photos / processed', st.photos + ' / ' + st.processed) + row('Last processing', st.last_ms + ' ms') +
      row('Detector', st.detector_on ? 'on' : 'OFF', st.detector_on ? 'ok' : 'bad') + row('Camera', !st.camera_ok ? 'NOT OPENED' : (st.camera_age_s > 1 ? 'NO NEW FRAMES' : 'ok'), (st.camera_ok && st.camera_age_s <= 1) ? 'ok' : 'bad');
    document.getElementById('surv').innerHTML = s.survivors.length ? s.survivors.map(x =>
      `<div class="row"><span>#${x.id} ${x.confirmed ? 'CONFIRMED' : 'tentative'} (${x.hits} sightings)</span><span>${x.lat.toFixed(6)}, ${x.lon.toFixed(6)}</span></div>`).join('') : 'none yet';
    document.getElementById('last').src = '/last.jpg?' + Date.now();
  } catch(e) {}
}
window.addEventListener('resize', () => { if (draw) draw(last); });
init();
</script></body></html>""".replace("__DRAW__", DRAW_JS)

REPORT_HTML = """<!doctype html><html><head><meta charset="utf-8"><title>Flight report</title>
<style>body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;display:flex;height:100vh}
#map{flex:2} #side{flex:1;padding:12px;overflow:auto;max-width:480px} .row{display:flex;justify-content:space-between;border-bottom:1px solid #333;padding:3px 0}</style></head><body>
<canvas id="map"></canvas><div id="side"><h2>Flight report</h2><div id="info"></div><h3>Survivors</h3><div id="surv"></div></div>
<script>
__DRAW__
const DATA = __DATA__;
const draw = makeMap(document.getElementById('map'), DATA.mission);
const state = {drone: null, survivors: DATA.survivors, track: DATA.track};
function render(){ draw(state); }
window.addEventListener('resize', render); render();
const row = (k, v) => `<div class="row"><span>${k}</span><span>${v}</span></div>`;
document.getElementById('info').innerHTML = row('Photos taken', DATA.photos) + row('Confirmed survivors', DATA.survivors.filter(s => s.confirmed).length) +
  row('All detections (merged)', DATA.survivors.length) + row('Planned path', Math.round(DATA.mission.est_path_m) + ' m') + row('Altitude / speed', DATA.mission.altitude_agl_m + ' m / ' + DATA.mission.speed_mps + ' m/s');
document.getElementById('surv').innerHTML = DATA.survivors.map(x => row('#' + x.id + (x.confirmed ? ' confirmed' : ' tentative') + ' (' + x.hits + ' sightings)', x.lat.toFixed(6) + ', ' + x.lon.toFixed(6))).join('') || 'none';
</script></body></html>"""
