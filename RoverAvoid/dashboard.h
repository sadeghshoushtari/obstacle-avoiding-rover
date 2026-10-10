#pragma once

const char DASHBOARD_HTML[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rover Control</title>
<style>
:root{--bg:#0f1419;--panel:#182029;--line:#2a3542;--text:#e6edf3;--muted:#8b98a5;
--go:#3fb950;--stop:#f85149;--warn:#d29922;--accent:#58a6ff;--obs:#ff7b72}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:15px/1.4 system-ui,-apple-system,Segoe UI,sans-serif;padding:16px}
header{display:flex;flex-wrap:wrap;align-items:center;gap:12px;margin-bottom:16px}
h1{font-size:22px;margin:0 6px 0 0}
.pill{padding:6px 14px;border-radius:999px;font-weight:700;background:var(--line);letter-spacing:.04em}
#conn{color:var(--muted);font-size:13px}
.btns{margin-left:auto;display:flex;gap:10px}
button{border:0;border-radius:10px;padding:12px 26px;font-size:17px;font-weight:700;color:#fff;cursor:pointer}
button:active{transform:scale(.97)}
#go{background:var(--go)}#halt{background:var(--stop)}#reset{background:var(--line);font-size:14px;padding:12px 16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(440px,100%),1fr));gap:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px}
.card h2{font-size:13px;margin:0 0 10px;color:var(--muted);font-weight:600;text-transform:uppercase;letter-spacing:.06em}
.wide{grid-column:1/-1}
canvas{width:100%;display:block}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;margin:16px 0}
.stat{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:12px 14px}
.stat b{display:block;font-size:26px;font-variant-numeric:tabular-nums}
.stat span{color:var(--muted);font-size:13px}
#log{font:13px/1.6 ui-monospace,Consolas,monospace;color:var(--muted);max-height:160px;overflow:auto;margin:0;padding:0;list-style:none}
#log li:first-child{color:var(--text)}
.sl{display:flex;align-items:center;gap:12px;margin:8px 0}
.sl label{width:95px;color:var(--muted)}
.sl input{flex:1;accent-color:var(--accent)}
.sl output{width:150px;text-align:right;font-variant-numeric:tabular-nums}
.pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,64px);gap:8px;justify-content:center;margin:6px 0 10px}
.pad button{padding:0;font-size:26px;background:var(--line);touch-action:none;user-select:none;-webkit-user-select:none}
.pad button.on{background:var(--accent)}
.pad .f{grid-area:1/2}.pad .l{grid-area:2/1}.pad .s{grid-area:2/2;background:var(--stop);font-size:15px}.pad .r{grid-area:2/3}.pad .b{grid-area:3/2}
.hint{color:var(--muted);font-size:13px;text-align:center;margin:0}
</style></head><body>
<header>
  <h1>Rover</h1><span class="pill" id="state">&ndash;</span><span id="conn">connecting&hellip;</span>
  <div class="btns"><button id="reset" title="Clear map and graph">Reset map</button>
  <button id="go">START</button><button id="halt">STOP</button></div>
</header>
<div class="stats">
  <div class="stat"><b id="sDist">&ndash;</b><span>Distance ahead</span></div>
  <div class="stat"><b id="sTrav">0 cm</b><span>Travelled</span></div>
  <div class="stat"><b id="sSpeed">0.0</b><span>Speed (cm/s)</span></div>
  <div class="stat"><b id="sAvoid">0</b><span>Obstacles avoided</span></div>
  <div class="stat"><b id="sUp">0 s</b><span>Uptime</span></div>
</div>
<div class="grid">
  <div class="card"><h2>Radar &middot; sensor view</h2><canvas id="radar" height="270"></canvas></div>
  <div class="card"><h2>Map &middot; path and detected obstacles</h2><canvas id="map" height="270"></canvas></div>
  <div class="card"><h2>Manual control</h2>
    <div class="pad">
      <button class="f" data-d="f" aria-label="Forward">&#9650;</button>
      <button class="l" data-d="l" aria-label="Turn left">&#9664;</button>
      <button class="s" id="mstop">STOP</button>
      <button class="r" data-d="r" aria-label="Turn right">&#9654;</button>
      <button class="b" data-d="b" aria-label="Backward">&#9660;</button>
    </div>
    <p class="hint">Hold a button (or an arrow key) to move. Forward stops by itself at the stop distance.
    Press START to go back to automatic.</p>
  </div>
  <div class="card"><h2>Speed &middot; raise until the motors click, then back off</h2>
    <div class="sl"><label for="vc">Driving</label><input id="vc" type="range" min="600" max="1800" step="50"><output id="oc"></output></div>
    <div class="sl"><label for="vt">Turning</label><input id="vt" type="range" min="600" max="1800" step="50"><output id="ot"></output></div>
  </div>
  <div class="card wide"><h2>Distance ahead &middot; last 30 s</h2><canvas id="chart" height="190"></canvas></div>
  <div class="card wide"><h2>Events</h2><ul id="log"></ul></div>
</div>
<script>
const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const C = {line:css('--line'), text:css('--text'), muted:css('--muted'), go:css('--go'),
  stop:css('--stop'), warn:css('--warn'), accent:css('--accent'), obs:css('--obs'), panel:css('--panel')};
const SENSOR_OFFSET = 8;      // cm from wheel axis to sensor
const RADAR_RANGE = 100;      // cm
const CHART_SECONDS = 30, CHART_MAX = 150;

let hist = [], path = [], obs = [], seen = new Set();
let last = null, lastT = 0, speed = 0, lastEid = -1, safe = 20, slidersSet = false;
const CM_PER_HALFSTEP = Math.PI * 6.8 / 4096;
const fmtSpeed = v => v + '  (' + (v * CM_PER_HALFSTEP).toFixed(1) + ' cm/s)';

const cv = {};
for (const id of ['radar', 'map', 'chart']) { const c = $(id); cv[id] = {c, H: +c.getAttribute('height')}; }
function ctx(id) {
  const o = cv[id], r = window.devicePixelRatio || 1, w = o.c.clientWidth;
  if (o.c.width !== Math.round(w * r)) { o.c.width = Math.round(w * r); o.c.height = Math.round(o.H * r); o.c.style.height = o.H + 'px'; }
  const x = o.c.getContext('2d'); x.setTransform(r, 0, 0, r, 0, 0); x.clearRect(0, 0, w, o.H);
  x.font = '12px system-ui,sans-serif'; return [x, w, o.H];
}
const rad = d => d * Math.PI / 180;

function worldPoint(j, a, d) {
  const sx = j.x - SENSOR_OFFSET * Math.sin(j.h), sy = j.y + SENSOR_OFFSET * Math.cos(j.h);
  const t = j.h + rad(a - 90);
  return [sx - d * Math.sin(t), sy + d * Math.cos(t)];
}
function addObstacle(p) {
  for (let i = Math.max(0, obs.length - 40); i < obs.length; i++)
    if (Math.hypot(obs[i][0] - p[0], obs[i][1] - p[1]) < 3) return;
  obs.push(p); if (obs.length > 1500) obs.shift();
}

function onData(j) {
  const now = performance.now() / 1000;
  safe = j.sf;
  if (last && now > lastT) speed = 0.6 * speed + 0.4 * Math.max(0, (j.tr - last.tr) / (now - lastT));
  const fresh = j.st === 'driving' || j.st === 'stopped' || j.st === 'manual';
  hist.push([now, fresh && j.d < 999 ? j.d : null]);
  while (hist.length && hist[0][0] < now - CHART_SECONDS) hist.shift();

  const lp = path[path.length - 1];
  if (!lp || Math.hypot(lp[0] - j.x, lp[1] - j.y) > 0.5) path.push([j.x, j.y]);
  if (j.st === 'driving' && j.d < 80) addObstacle(worldPoint(j, 90, j.d));
  for (const [a, d] of j.scan) {
    const key = j.sid + ':' + a;
    if (!seen.has(key)) { seen.add(key); if (d < 150) addObstacle(worldPoint(j, a, d)); }
  }

  if (j.eid !== lastEid && j.ev) {
    lastEid = j.eid;
    const li = document.createElement('li');
    li.textContent = new Date().toLocaleTimeString() + '  ' + j.ev;
    $('log').prepend(li);
    while ($('log').children.length > 50) $('log').lastChild.remove();
  }

  const colors = {driving: C.go, scanning: C.warn, turning: C.accent, backing: C.accent, manual: C.accent, stopped: C.line};
  $('state').textContent = j.st.toUpperCase();
  $('state').style.background = colors[j.st] || C.line;
  $('sDist').textContent = j.d >= 999 ? '> 4 m' : j.d + ' cm';
  $('sTrav').textContent = j.tr < 1000 ? Math.round(j.tr) + ' cm' : (j.tr / 100).toFixed(1) + ' m';
  $('sSpeed').textContent = speed.toFixed(1);
  $('sAvoid').textContent = j.av;
  $('sUp').textContent = j.up < 60 ? j.up + ' s' : Math.floor(j.up / 60) + ' min ' + (j.up % 60) + ' s';

  if (!slidersSet) {
    slidersSet = true;
    $('vc').value = j.cs; $('vt').value = j.ts;
    $('oc').textContent = fmtSpeed(j.cs); $('ot').textContent = fmtSpeed(j.ts);
  }
  last = j; lastT = now;
  drawRadar(j); drawMap(j); drawChart(now);
}

function drawRadar(j) {
  const [x, w, H] = ctx('radar');
  const cx = w / 2, cy = H - 18, R = Math.min(w / 2 - 30, H - 50);
  const P = (a, d) => [cx + R * d / RADAR_RANGE * Math.cos(rad(a)), cy - R * d / RADAR_RANGE * Math.sin(rad(a))];
  x.fillStyle = 'rgba(88,166,255,.08)'; x.beginPath(); x.moveTo(cx, cy);
  x.arc(cx, cy, R, -rad(120), -rad(60)); x.closePath(); x.fill();
  x.strokeStyle = C.line; x.fillStyle = C.muted; x.lineWidth = 1;
  for (const d of [25, 50, 75, 100]) {
    x.beginPath(); x.arc(cx, cy, R * d / RADAR_RANGE, Math.PI, 2 * Math.PI); x.stroke();
    x.fillText(d + ' cm', cx + 4, cy - R * d / RADAR_RANGE + 13);
  }
  for (let a = 0; a <= 180; a += 30) { const [px, py] = P(a, RADAR_RANGE); x.beginPath(); x.moveTo(cx, cy); x.lineTo(px, py); x.stroke(); }
  x.fillText('LEFT', 4, cy + 14); x.fillText('RIGHT', w - 40, cy + 14);
  x.strokeStyle = C.stop; x.setLineDash([5, 5]);
  x.beginPath(); x.arc(cx, cy, R * safe / RADAR_RANGE, Math.PI, 2 * Math.PI); x.stroke(); x.setLineDash([]);
  const [sx, sy] = P(j.a, RADAR_RANGE);
  const g = x.createLinearGradient(cx, cy, sx, sy); g.addColorStop(0, 'rgba(63,185,80,.9)'); g.addColorStop(1, 'rgba(63,185,80,.1)');
  x.strokeStyle = g; x.lineWidth = 3; x.beginPath(); x.moveTo(cx, cy); x.lineTo(sx, sy); x.stroke(); x.lineWidth = 1;
  x.textAlign = 'center';
  for (const [a, d] of j.scan) {
    const [px, py] = P(a, Math.min(d, RADAR_RANGE));
    x.fillStyle = d < safe ? C.stop : (d > RADAR_RANGE ? C.muted : C.warn);
    x.beginPath(); x.arc(px, py, 5, 0, 2 * Math.PI); x.fill();
    const [lx, ly] = P(a, RADAR_RANGE * 1.12);
    x.fillText(d >= 999 ? 'clear' : d > RADAR_RANGE ? d : d + '', lx, ly + 4);
  }
  x.textAlign = 'left';
  if ((j.st === 'driving' || j.st === 'stopped' || j.st === 'manual') && j.d < RADAR_RANGE) {
    const [px, py] = P(90, j.d);
    x.fillStyle = j.d < safe ? C.stop : C.go; x.beginPath(); x.arc(px, py, 7, 0, 2 * Math.PI); x.fill();
  }
  x.fillStyle = C.text; x.beginPath(); x.moveTo(cx - 9, cy + 6); x.lineTo(cx + 9, cy + 6); x.lineTo(cx, cy - 8); x.closePath(); x.fill();
}

function drawMap(j) {
  const [x, w, H] = ctx('map');
  let minX = j.x - 40, maxX = j.x + 40, minY = j.y - 40, maxY = j.y + 40;
  for (const p of path.concat(obs)) { minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]); minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]); }
  const pad = 16, s = Math.min((w - 2 * pad) / (maxX - minX), (H - 2 * pad) / (maxY - minY));
  const ox = (w - s * (maxX - minX)) / 2, oy = (H - s * (maxY - minY)) / 2;
  const M = (px, py) => [ox + (px - minX) * s, H - oy - (py - minY) * s];
  const step = 50; x.strokeStyle = C.line; x.lineWidth = 1;
  for (let gx = Math.ceil(minX / step) * step; gx <= maxX; gx += step) { const [a] = M(gx, 0); x.beginPath(); x.moveTo(a, 0); x.lineTo(a, H); x.stroke(); }
  for (let gy = Math.ceil(minY / step) * step; gy <= maxY; gy += step) { const [, b] = M(0, gy); x.beginPath(); x.moveTo(0, b); x.lineTo(w, b); x.stroke(); }
  x.fillStyle = C.muted; x.fillText('grid 50 cm', 6, 14);
  const [s0x, s0y] = M(0, 0); x.strokeStyle = C.muted; x.beginPath(); x.arc(s0x, s0y, 5, 0, 2 * Math.PI); x.stroke();
  x.fillStyle = C.obs;
  for (const p of obs) { const [a, b] = M(p[0], p[1]); x.fillRect(a - 2, b - 2, 4, 4); }
  x.strokeStyle = C.accent; x.lineWidth = 2; x.beginPath();
  path.forEach((p, i) => { const [a, b] = M(p[0], p[1]); i ? x.lineTo(a, b) : x.moveTo(a, b); }); x.stroke();
  const [rx, ry] = M(j.x, j.y), fx = -Math.sin(j.h), fy = Math.cos(j.h), k = 11;
  x.fillStyle = C.go; x.beginPath();
  x.moveTo(rx + fx * k, ry - fy * k);
  x.lineTo(rx - fx * k * .6 + fy * k * .6, ry + fy * k * .6 + fx * k * .6);
  x.lineTo(rx - fx * k * .6 - fy * k * .6, ry + fy * k * .6 - fx * k * .6);
  x.closePath(); x.fill();
}

function drawChart(now) {
  const [x, w, H] = ctx('chart');
  const L = 44, Rm = 10, T = 8, B = 22, pw = w - L - Rm, ph = H - T - B;
  const X = t => L + pw * (1 - (now - t) / CHART_SECONDS), Y = d => T + ph * (1 - Math.min(d, CHART_MAX) / CHART_MAX);
  x.strokeStyle = C.line; x.fillStyle = C.muted; x.lineWidth = 1;
  for (const d of [0, 50, 100, 150]) { x.beginPath(); x.moveTo(L, Y(d)); x.lineTo(w - Rm, Y(d)); x.stroke(); x.fillText(d + ' cm', 2, Y(d) + 4); }
  for (const s of [30, 20, 10, 0]) x.fillText(s ? '-' + s + ' s' : 'now', X(now - s) - (s ? 14 : 24), H - 5);
  x.strokeStyle = C.stop; x.setLineDash([5, 5]); x.beginPath(); x.moveTo(L, Y(safe)); x.lineTo(w - Rm, Y(safe)); x.stroke(); x.setLineDash([]);
  x.fillStyle = C.stop; x.fillText('stop at ' + safe + ' cm', w - Rm - 80, Y(safe) - 5);
  x.strokeStyle = C.accent; x.lineWidth = 2; x.beginPath(); let pen = false;
  for (const [t, d] of hist) { if (d === null) { pen = false; continue; } const [a, b] = [X(t), Y(d)]; pen ? x.lineTo(a, b) : x.moveTo(a, b); pen = true; }
  x.stroke();
}

async function poll() {
  try {
    const r = await fetch('/data', {cache: 'no-store'});
    onData(await r.json());
    $('conn').textContent = 'connected'; $('conn').style.color = C.go;
  } catch (e) { $('conn').textContent = 'no connection'; $('conn').style.color = C.stop; }
  setTimeout(poll, 300);
}
for (const [id, out, key] of [['vc', 'oc', 'c'], ['vt', 'ot', 't']]) {
  $(id).oninput = () => $(out).textContent = fmtSpeed(+$(id).value);
  $(id).onchange = () => fetch('/set?' + key + '=' + $(id).value);
}
let held = null, holdTimer = null;
function press(d) {
  if (held === d) return;
  release(); held = d;
  document.querySelectorAll('.pad [data-d]').forEach(b => b.classList.toggle('on', b.dataset.d === d));
  const send = () => fetch('/m?d=' + d).catch(() => {});
  send(); holdTimer = setInterval(send, 250);
}
function release() {
  if (!held) return;
  held = null; clearInterval(holdTimer);
  document.querySelectorAll('.pad [data-d]').forEach(b => b.classList.remove('on'));
  fetch('/m?d=s').catch(() => {});
}
document.querySelectorAll('.pad [data-d]').forEach(b => {
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); press(b.dataset.d); });
  for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(ev, release);
});
$('mstop').onclick = () => { release(); fetch('/stop'); };
const KEYS = {ArrowUp: 'f', ArrowDown: 'b', ArrowLeft: 'l', ArrowRight: 'r'};
addEventListener('keydown', e => { if (KEYS[e.key] && e.target.tagName !== 'INPUT') { e.preventDefault(); if (!e.repeat) press(KEYS[e.key]); } });
addEventListener('keyup', e => { if (KEYS[e.key] && held === KEYS[e.key]) release(); });
addEventListener('blur', release);
$('go').onclick = () => fetch('/start');
$('halt').onclick = () => fetch('/stop');
$('reset').onclick = () => { hist = []; path = []; obs = []; seen = new Set(); fetch('/reset'); };
poll();
</script></body></html>)HTML";
