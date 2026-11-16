// FS Lap Sim front end. Plain JavaScript, no build step, no external libraries.
// The physics runs in Python inside a web worker (see worker.js).

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const COL = { A: css("--a"), B: css("--b"), ink: css("--ink"), ink2: css("--ink2"), ink3: css("--ink3"), line: css("--line"), bg: css("--bg"), surface: css("--surface") };
const SEQ = ["#27446f", "#3987E5", "#cfe6ff"];          // one hue, dark to light
const DIV = ["#e8704a", "#6f746d", "#4aa3ff"];          // negative, neutral, positive

const state = {
  schema: null, tracks: {}, trackId: "fsuk_autocross",
  setups: { A: null, B: null }, active: "A", compare: false,
  results: { A: null, B: null }, cursor: null, corner: 1, colour: "v",
};
let workers = {}, ready = 0, reqId = 0, pending = {}, runTimer = null, mapGeom = null;

// ---------- helpers ----------
const kmh = (v) => v * 3.6;
const fmt = (x, d = 1) => (x == null || isNaN(x) ? "–" : x.toFixed(d));
const fmtTime = (t) => (t >= 100 ? `${Math.floor(t / 60)}:${(t % 60).toFixed(2).padStart(5, "0")}` : t.toFixed(2));
const signed = (x, d = 2) => (x > 0 ? "+" : x < 0 ? "−" : "±") + Math.abs(x).toFixed(d);
const lerp = (a, b, t) => a + (b - a) * t;
function hex(c) { return [1, 3, 5].map((i) => parseInt(c.slice(i, i + 2), 16)); }
function ramp(stops, t) {
  t = Math.min(1, Math.max(0, t)) * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(t)), f = t - i, a = hex(stops[i]), b = hex(stops[i + 1]);
  return `rgb(${a.map((v, k) => Math.round(lerp(v, b[k], f))).join(",")})`;
}
function setupCanvas(cv) {
  const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}
const shown = () => state.results[state.compare ? state.active : "A"] || state.results.A;
const shownKey = () => (state.compare && state.results[state.active] ? state.active : "A");

// ---------- colour channels for the map ----------
const CHANNELS = {
  v: { label: "Speed", unit: "km/h", get: (c, i) => kmh(c.v[i]), div: false, d: 0 },
  ay: { label: "Cornering", unit: "g", get: (c, i) => Math.abs(c.ay[i]), div: false, zero: true, d: 2 },
  ax: { label: "Braking (−) / acceleration (+)", unit: "g", get: (c, i) => c.ax[i], div: true, d: 2 },
  p: { label: "Battery power, regen (−) / drive (+)", unit: "kW", get: (c, i) => c.p[i], div: true, d: 0 },
  grip: { label: "Share of tyre grip in use", unit: "%", get: (c, i) => 100 * Math.min(1, Math.max(c.uf[i], c.ur[i])), div: false, zero: true, d: 0 },
};
function channelScale(res, key) {
  const ch = CHANNELS[key], n = res.ch.v.length;
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < n; i++) { const x = ch.get(res.ch, i); if (x < lo) lo = x; if (x > hi) hi = x; }
  if (ch.div) { const m = Math.max(Math.abs(lo), Math.abs(hi)) || 1; lo = -m; hi = m; }
  else if (ch.zero) lo = 0;
  if (hi - lo < 1e-9) hi = lo + 1;
  return { lo, hi, colour: (x) => ramp(ch.div ? DIV : SEQ, (x - lo) / (hi - lo)) };
}

// ---------- track map ----------
function drawMap() {
  const cv = $("map"), { ctx, w, h } = setupCanvas(cv);
  const tr = state.tracks[state.trackId], res = shown();
  if (!tr) return;
  const xs = tr.x, ys = tr.y, n = xs.length, pad = 34;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (let i = 0; i < n; i++) { x0 = Math.min(x0, xs[i]); x1 = Math.max(x1, xs[i]); y0 = Math.min(y0, ys[i]); y1 = Math.max(y1, ys[i]); }
  // lay long, narrow tracks (the acceleration strip) on their side
  const rot = (y1 - y0) > 1.4 * (x1 - x0 + 1) && w > h;
  // a true 90 degree turn (not a mirror image), so left stays left
  const gx = (i) => (rot ? ys[i] : xs[i]), gy = (i) => (rot ? -xs[i] : ys[i]);
  const [a0, a1, b0, b1] = rot ? [y0, y1, -x1, -x0] : [x0, x1, y0, y1];
  const sc = Math.min((w - 2 * pad) / Math.max(a1 - a0, 1), (h - 2 * pad) / Math.max(b1 - b0, 1));
  const ox = (w - sc * (a1 - a0)) / 2, oy = (h - sc * (b1 - b0)) / 2;
  const px = (i) => ox + (gx(i) - a0) * sc, py = (i) => h - oy - (gy(i) - b0) * sc;
  const P = new Array(n);
  for (let i = 0; i < n; i++) P[i] = [px(i), py(i)];
  mapGeom = { P, closed: tr.closed, labels: [] };

  ctx.lineCap = "round"; ctx.lineJoin = "round";
  ctx.strokeStyle = "#2a2c2a"; ctx.lineWidth = 11;
  ctx.beginPath(); P.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
  if (tr.closed) ctx.closePath();
  ctx.stroke();
  if (!res) return;

  const ch = CHANNELS[state.colour], scale = channelScale(res, state.colour), m = res.ch.v.length;
  ctx.lineWidth = 4.5;
  for (let i = 0; i < m; i++) {
    const a = P[i], b = P[(i + 1) % n];
    ctx.strokeStyle = scale.colour(ch.get(res.ch, i));
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
  }
  // start line marker
  ctx.fillStyle = COL.ink; ctx.beginPath(); ctx.arc(P[0][0], P[0][1], 4, 0, 7); ctx.fill();
  ctx.font = "11px Inter, system-ui, sans-serif"; ctx.fillStyle = COL.ink2; ctx.textAlign = "left"; ctx.textBaseline = "middle";
  ctx.fillText("Start", P[0][0] + 9, P[0][1] - 9);

  // selected corner highlight + numbered labels pushed to the outside of the bend
  const sel = res.corners.find((c) => c.n === state.corner);
  if (sel) {
    ctx.strokeStyle = COL.ink; ctx.lineWidth = 9; ctx.globalAlpha = 0.28;
    ctx.beginPath();
    for (let i = sel.start_i; i <= sel.end_i + 1; i++) { const p = P[i % n]; i === sel.start_i ? ctx.moveTo(p[0], p[1]) : ctx.lineTo(p[0], p[1]); }
    ctx.stroke(); ctx.globalAlpha = 1;
  }
  ctx.textAlign = "center";
  for (const c of res.corners) {
    const i = c.apex_i, a = P[(i - 2 + n) % n], b = P[(i + 2) % n];
    let nx = -(b[1] - a[1]), ny = b[0] - a[0];
    const len = Math.hypot(nx, ny) || 1; nx /= len; ny /= len;
    // canvas y points down: this normal points right of travel. Outside of a left turn is the right-hand side.
    const out = c.dir === "left" ? 1 : -1;
    const lx = P[i][0] + nx * out * 15, ly = P[i][1] + ny * out * 15, on = c.n === state.corner;
    ctx.beginPath(); ctx.arc(lx, ly, 8, 0, 7);
    ctx.fillStyle = on ? COL.ink : COL.surface; ctx.fill();
    ctx.strokeStyle = on ? COL.ink : COL.ink3; ctx.lineWidth = 1; ctx.stroke();
    ctx.fillStyle = on ? COL.bg : COL.ink2; ctx.font = (on ? "600 " : "") + "10px Inter, system-ui, sans-serif";
    ctx.fillText(c.n, lx, ly + 0.5);
    mapGeom.labels.push({ x: lx, y: ly, n: c.n });
  }
  // cursor
  if (state.cursor != null && state.cursor < m) {
    const p = P[state.cursor];
    ctx.beginPath(); ctx.arc(p[0], p[1], 7, 0, 7); ctx.fillStyle = COL.bg; ctx.fill();
    ctx.beginPath(); ctx.arc(p[0], p[1], 5, 0, 7); ctx.fillStyle = COL.ink; ctx.fill();
  }
  $("legend").innerHTML = `<span>${fmt(scale.lo, ch.d)}</span><div class="bar" style="background:linear-gradient(90deg,${(ch.div ? DIV : SEQ).join(",")})"></div><span>${fmt(scale.hi, ch.d)} ${ch.unit}</span><span>· ${ch.label}</span>`;
}

function mapPointer(e, click) {
  if (!mapGeom || !shown()) return;
  const r = $("map").getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (click) {
    const hit = mapGeom.labels.find((l) => Math.hypot(l.x - x, l.y - y) < 12);
    if (hit) return selectCorner(hit.n);
  }
  let best = -1, bd = 1e9;
  const m = shown().ch.v.length;
  for (let i = 0; i < m; i++) { const p = mapGeom.P[i], d = (p[0] - x) ** 2 + (p[1] - y) ** 2; if (d < bd) { bd = d; best = i; } }
  if (bd > 40 * 40) { setCursor(null); return; }
  if (click) {
    const c = shown().corners.find((c) => best >= c.start_i - 4 && best <= c.end_i + 4);
    if (c) selectCorner(c.n);
  } else setCursor(best, { x, y });
}

function setCursor(i, at) {
  state.cursor = i;
  const tip = $("tip"), res = shown();
  if (i == null || !res || !at) tip.hidden = true;
  else {
    const c = res.ch, k = shownKey(), other = state.compare && state.results[k === "A" ? "B" : "A"];
    tip.innerHTML = `<span>Distance</span> <b>${fmt(i * res.ds, 0)} m</b><br><span>Speed</span> <b>${fmt(kmh(c.v[i]), 1)} km/h</b>` +
      (other ? ` <span>(${k === "A" ? "B" : "A"}: ${fmt(kmh(other.ch.v[i]), 1)})</span>` : "") +
      `<br><span>Cornering</span> <b>${fmt(Math.abs(c.ay[i]), 2)} g</b> <span>${c.ay[i] > 0.05 ? "left" : c.ay[i] < -0.05 ? "right" : ""}</span>` +
      `<br><span>${c.ax[i] >= 0 ? "Accelerating" : "Braking"}</span> <b>${fmt(Math.abs(c.ax[i]), 2)} g</b>` +
      `<br><span>Battery power</span> <b>${fmt(c.p[i], 1)} kW</b><br><span>Grip in use</span> <b>${fmt(100 * Math.max(c.uf[i], c.ur[i]), 0)}%</b>`;
    tip.hidden = false;
    const wrap = $("map").clientWidth;
    tip.style.left = Math.min(at.x + 14, wrap - tip.offsetWidth - 4) + "px";
    tip.style.top = Math.max(4, at.y - tip.offsetHeight - 10) + "px";
  }
  drawMap(); drawCharts();
}

// ---------- line charts ----------
const CHARTS = [
  { key: "v", title: "Speed", unit: "km/h", get: (c) => c.v.map(kmh), d: 1 },
  { key: "ay", title: "Cornering acceleration (left +, right −)", unit: "g", get: (c) => c.ay, d: 2, zero: true },
  { key: "ax", title: "Acceleration (+) and braking (−)", unit: "g", get: (c) => c.ax, d: 2, zero: true },
  { key: "p", title: "Battery power (negative = regen charging)", unit: "kW", get: (c) => c.p, d: 1, zero: true },
  { key: "dt", title: "Time gained or lost by B relative to A (above zero = B is behind)", unit: "s", cmp: true, d: 2, zero: true,
    get: (c, all) => all.A.ch.t.map((t, i) => (all.B.ch.t[i] ?? NaN) - t), single: true },
];
const RIDE_CHARTS = [
  { key: "roll_dyn", title: "Body roll (leaning right +, left −)", unit: "deg", get: (c) => c.roll_dyn, d: 2, zero: true },
  { key: "pitch", title: "Pitch (nose down under braking +, nose up under power −)", unit: "deg", get: (c) => c.pitch, d: 2, zero: true },
  { key: "dv_fl", title: "Front-left damper speed (compressing +, extending −)", unit: "mm/s", get: (c) => c.dv_fl, d: 0, zero: true },
  { key: "dfz", title: "Front-left tyre load: time-based model minus settled model", unit: "N", get: (c) => c.dfz_fl, d: 0, zero: true },
];
function niceTicks(lo, hi, n = 4) {
  const span = hi - lo || 1, step0 = span / n, mag = 10 ** Math.floor(Math.log10(step0));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => span / s <= n + 0.5) || mag * 10;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
  return out;
}
function drawLineChart(cv, o) {
  const { ctx, w, h } = setupCanvas(cv);
  const L = 44, R = 10, T = 6, B = o.xlabels ? 20 : 6, n = o.n;
  let lo = Infinity, hi = -Infinity;
  for (const s of o.series) for (const v of s.data) { if (v < lo) lo = v; if (v > hi) hi = v; }
  if (o.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  const padv = (hi - lo) * 0.08 || 0.5; lo -= padv; hi += padv;
  const X = (i) => L + (i / Math.max(1, n - 1)) * (w - L - R), Y = (v) => T + (1 - (v - lo) / (hi - lo)) * (h - T - B);
  // corner bands
  for (const b of o.bands || []) {
    ctx.fillStyle = b.on ? "rgba(236,238,233,0.13)" : "rgba(236,238,233,0.04)";
    ctx.fillRect(X(b.a), T, Math.max(1, X(b.b) - X(b.a)), h - T - B);
  }
  ctx.font = "10.5px Inter, system-ui, sans-serif"; ctx.textBaseline = "middle"; ctx.textAlign = "right";
  for (const t of niceTicks(lo, hi)) {
    ctx.strokeStyle = t === 0 && o.zero ? "#4a4d4a" : "#262826"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(L, Math.round(Y(t)) + 0.5); ctx.lineTo(w - R, Math.round(Y(t)) + 0.5); ctx.stroke();
    ctx.fillStyle = COL.ink3; ctx.fillText(String(t), L - 6, Y(t));
  }
  if (o.xlabels) {
    ctx.textAlign = "center"; ctx.textBaseline = "top";
    for (const t of niceTicks(0, (n - 1) * o.dx, 8)) { ctx.fillText(o.xfmt ? o.xfmt(t) : String(t), X(t / o.dx), h - B + 5); }
  }
  ctx.lineJoin = "round"; ctx.lineWidth = 2;
  for (const s of o.series) {
    ctx.strokeStyle = s.colour; ctx.beginPath();
    let pen = false;
    s.data.forEach((v, i) => { if (isNaN(v)) { pen = false; return; } pen ? ctx.lineTo(X(i), Y(v)) : ctx.moveTo(X(i), Y(v)); pen = true; });
    ctx.stroke();
    if (o.dots) { ctx.fillStyle = s.colour; s.data.forEach((v, i) => { ctx.beginPath(); ctx.arc(X(i), Y(v), 2.5, 0, 7); ctx.fill(); }); }
  }
  if (o.cursor != null && o.cursor < n) {
    const x = X(o.cursor);
    ctx.strokeStyle = COL.ink3; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, h - B); ctx.stroke();
    for (const s of o.series) {
      const v = s.data[o.cursor]; if (v == null || isNaN(v)) continue;
      ctx.beginPath(); ctx.arc(x, Y(v), 5, 0, 7); ctx.fillStyle = COL.surface; ctx.fill();
      ctx.beginPath(); ctx.arc(x, Y(v), 3.5, 0, 7); ctx.fillStyle = s.colour; ctx.fill();
    }
  }
  cv._idx = (clientX) => { const r = cv.getBoundingClientRect(); return Math.round(((clientX - r.left - L) / (r.width - L - R)) * (n - 1)); };
}

function chartBlock(host, id, title) {
  let el = host.querySelector(`[data-id="${id}"]`);
  if (!el) {
    el = document.createElement("div"); el.className = "chart"; el.dataset.id = id;
    el.innerHTML = `<div class="ct"><span></span><em></em></div><canvas></canvas>`;
    host.appendChild(el);
  }
  el.querySelector("span").textContent = title;
  return el;
}

function drawSet(host, defs) {
  const all = state.results, A = all.A;
  const both = state.compare && all.B, n = A.ch.v.length;
  const bands = A.corners.map((c) => ({ a: c.start_i, b: c.end_i, on: c.n === state.corner }));
  const list = defs.filter((c) => !c.cmp || both);
  host.querySelectorAll(".chart").forEach((el) => { if (!list.some((c) => c.key === el.dataset.id)) el.remove(); });
  list.forEach((c, idx) => {
    const el = chartBlock(host, c.key, `${c.title}, ${c.unit}`), cv = el.querySelector("canvas");
    const series = c.single ? [{ data: c.get(null, all), colour: COL.ink2 }]
      : [{ data: c.get(A.ch), colour: COL.A }].concat(both ? [{ data: c.get(all.B.ch), colour: COL.B }] : []);
    drawLineChart(cv, { n, series, bands, cursor: state.cursor, zero: c.zero, xlabels: idx === list.length - 1, dx: A.ds });
    const i = state.cursor;
    el.querySelector("em").textContent = i == null || i >= n ? "" :
      `${fmt(i * A.ds, 0)} m:  ` + series.map((s, k) => (c.single ? "" : both ? (k ? "B " : "A ") : "") + fmt(s.data[i], c.d)).join("   ") + ` ${c.unit}`;
    if (!cv._wired) {
      cv._wired = true;
      cv.addEventListener("mousemove", (e) => { const j = cv._idx(e.clientX); state.cursor = j >= 0 && j < state.results.A.ch.v.length ? j : null; $("tip").hidden = true; drawMap(); drawCharts(); });
      cv.addEventListener("mouseleave", () => setCursor(null));
      cv.addEventListener("click", (e) => { const j = cv._idx(e.clientX), cc = state.results.A.corners.find((q) => j >= q.start_i && j <= q.end_i); if (cc) selectCorner(cc.n); });
    }
  });
}

function drawCharts() {
  const all = state.results, A = all.A;
  if (!A) return;
  const both = state.compare && all.B;
  drawSet($("charts"), CHARTS);
  drawSet($("rideCharts"), RIDE_CHARTS);
  $("chartLegend").innerHTML = both ? `<span><i class="sw a"></i>Setup A</span><span><i class="sw b"></i>Setup B</span>` : "";

  // endurance lap-by-lap
  const lapsCard = $("lapsCard"), lh = $("lapCharts");
  lapsCard.hidden = !(A.lap_rows && A.lap_rows.length);
  if (!lapsCard.hidden) {
    const mk = (key, title, get) => {
      const el = chartBlock(lh, key, title), nl = A.laps;
      const padTo = (rows) => Array.from({ length: nl }, (_, i) => (rows[i] ? get(rows[i]) : NaN));
      const series = [{ data: padTo(A.lap_rows), colour: COL.A }].concat(both && all.B.lap_rows.length ? [{ data: padTo(all.B.lap_rows), colour: COL.B }] : []);
      drawLineChart(el.querySelector("canvas"), { n: nl, series, dots: true, xlabels: true, dx: 1, xfmt: (t) => "Lap " + (t + 1), zero: key === "soc" });
    };
    mk("lapt", "Lap time, s (lap 1 is from a standing start)", (r) => r.time);
    mk("soc", "Battery charge at the end of each lap, %", (r) => r.soc_end);
  }
}

// ---------- suspension and damper panel ----------
function drawBars(cv, labels, series, unit) {
  const { ctx, w, h } = setupCanvas(cv);
  const L = 8, R = 8, T = 18, B = 22, n = labels.length, k = series.length;
  const hi = Math.max(1, ...series.flatMap((s) => s.data)) * 1.12;
  const gw = (w - L - R) / n, bw = Math.min(34, (gw - 14) / k - 2);
  ctx.font = "10.5px Inter, system-ui, sans-serif"; ctx.textAlign = "center";
  ctx.strokeStyle = "#4a4d4a"; ctx.beginPath(); ctx.moveTo(L, h - B + 0.5); ctx.lineTo(w - R, h - B + 0.5); ctx.stroke();
  labels.forEach((lab, i) => {
    const cx = L + gw * (i + 0.5);
    ctx.fillStyle = COL.ink3; ctx.textBaseline = "top"; ctx.fillText(lab, cx, h - B + 6);
    series.forEach((s, j) => {
      const v = s.data[i], bh = (v / hi) * (h - T - B), x = cx - (k * (bw + 2)) / 2 + j * (bw + 2) + 1, y = h - B - bh;
      ctx.fillStyle = s.colour; ctx.beginPath(); ctx.roundRect(x, y, bw, bh, [4, 4, 0, 0]); ctx.fill();
      ctx.fillStyle = COL.ink2; ctx.textBaseline = "bottom"; ctx.fillText(v.toFixed(v < 10 ? 1 : 0) + unit, x + bw / 2, y - 3);
    });
  });
}
function drawRide() {
  const A = state.results.A, B = state.compare ? state.results.B : null;
  if (!A || !A.ride) return;
  const r = A.ride, rb = B && B.ride;
  const tiles = [
    ["Time lost to body movement", (q) => signed(q.time_cost, 2), "s", "Lap time with the time-based tyre loads fed back into grip, minus lap time if the body settled instantly. For endurance this is for one flying lap."],
    ["Lap if the body settled instantly", (q) => fmtTime(q.time_settled), "s", "The lap time from the settled model alone, with no spring and damper effects."],
    ["Ride frequency, front / rear", (q) => `${fmt(q.freq_f, 2)} / ${fmt(q.freq_r, 2)}`, "Hz", "How fast the body bounces on its springs. FS cars with wings usually sit between 3 and 4.5 Hz."],
    ["Damping ratio, front / rear", (q) => `${fmt(q.zeta_f, 2)} / ${fmt(q.zeta_r, 2)}`, "", "1.0 is the least damping that stops bouncing with no overshoot. 0.6 to 0.8 is a common target."],
    ["Roll damping ratio", (q) => fmt(q.zeta_roll, 2), "", "Below 1 the body overshoots its lean on turn-in, then settles."],
    ["Roll per g of cornering", (q) => fmt(q.roll_gradient, 2), "deg/g", "Roll gradient: how far the body leans for each g of cornering."],
    ["Peak roll", (q) => fmt(q.roll_max, 2), "deg", "Largest lean angle reached on this lap."],
    ["Peak dive / squat", (q) => `${fmt(q.dive_max, 2)} / ${fmt(q.squat_max, 2)}`, "deg", "Nose down under braking, nose up under power."],
    ["Peak damper speed, front / rear", (q) => `${fmt(q.dv_f_max, 0)} / ${fmt(q.dv_r_max, 0)}`, "mm/s", "Fastest suspension movement caused by the driver's inputs."],
    ["Most suspension travel used, front / rear", (q) => `${fmt(q.sus_f_max, 1)} / ${fmt(q.sus_r_max, 1)}`, "mm", "Compression from the standing ride height, from downforce, braking and squat."],
  ];
  $("rideKpis").innerHTML = tiles.map(([l, f, u, tip]) => `<div class="kpi small" title="${tip}"><div class="l">${l}</div><div class="v">${f(r)}<small>${u}</small></div>` +
    `<div class="d">${rb ? `<i class="sw b"></i>B ${f(rb)}` : ""}</div></div>`).join("");
  const labels = r.bins.map((b, i) => `${i ? r.bins[i - 1] : 0} to ${b}`).concat([`over ${r.bins[r.bins.length - 1]}`]);
  for (const [id, key] of [["histF", "hist_f"], ["histR", "hist_r"]])
    drawBars($(id), labels, [{ data: r[key], colour: COL.A }].concat(rb ? [{ data: rb[key], colour: COL.B }] : []), "%");
}

// ---------- KPI tiles ----------
function drawKpis() {
  const A = state.results.A, B = state.compare ? state.results.B : null;
  if (!A) return;
  const kind = A.kind;
  const timeLabel = { accel: "75 m time", skidpad: "Skidpad lap", autocross: "Lap time (standing start)", endurance: `Total time, ${A.laps} laps`, circuit: "Lap time (flying)" }[kind];
  const tiles = [
    [timeLabel, (r) => fmtTime(r.time), "s", (r) => r.time, 2, true],
    ["Top speed", (r) => fmt(kmh(r.v_max), 1), "km/h", (r) => kmh(r.v_max), 1],
    ["Peak cornering", (r) => fmt(r.ay_max, 2), "g", (r) => r.ay_max, 2],
    ["Peak braking", (r) => fmt(-r.ax_min, 2), "g", (r) => -r.ax_min, 2],
    ["Energy used (net)", (r) => fmt(r.energy_kwh, 2), "kWh", (r) => r.energy_kwh, 2],
    ["Battery at finish", (r) => fmt(r.soc_end, 1), "%", (r) => r.soc_end, 1],
  ];
  if (kind === "endurance") tiles.splice(1, 0, ["Flying lap", (r) => fmtTime(r.lap_time), "s", (r) => r.lap_time, 2]);
  $("kpis").innerHTML = tiles.map(([l, f, u, raw, d]) => `<div class="kpi"><div class="l">${l}</div><div class="v">${f(A)}<small>${u}</small></div>` +
    `<div class="d">${B ? `<i class="sw b"></i>B ${f(B)} (${signed(raw(B) - raw(A), d)})` : ""}</div></div>`).join("");
  const msgs = [];
  for (const [k, r] of [["A", A], ["B", B]]) if (r && !r.finished)
    msgs.push(`Setup ${k} runs out of battery after ${r.laps_done} of ${r.laps} laps, so the total time is not a finishing time. Lower the power limit under Conditions (teams commonly run 25 to 45 kW in endurance), raise regen, or fit a bigger battery.`);
  $("notice").hidden = !msgs.length; $("notice").innerHTML = msgs.join("<br>");
}

// ---------- corner details ----------
function selectCorner(n) {
  const A = state.results.A; if (!A || !A.corners.length) return;
  state.corner = ((n - 1 + A.corners.length) % A.corners.length) + 1;
  drawCorner(); drawMap(); drawCharts(); drawTable();
}
function drawCorner() {
  const A = state.results.A, B = state.compare ? state.results.B : null, host = $("corner");
  if (!A) return;
  $("cornerCard").hidden = $("tableCard").hidden = false;
  if (!A.corners.length) {
    $("cornerTitle").textContent = "Corner details";
    host.innerHTML = `<div class="empty">${A.kind === "accel" ? "The acceleration run is a straight line, so there are no corners to show." : "Skidpad is one constant-radius circle each way. Use the map and charts to read the steady cornering state."}</div>`;
    $("tableCard").hidden = true; return;
  }
  if (state.corner > A.corners.length) state.corner = 1;
  const a = A.corners[state.corner - 1], b = B && B.corners[state.corner - 1];
  $("cornerTitle").textContent = `Corner ${a.n}: ${a.dir}-hander, ${fmt(a.radius, 1)} m radius at its tightest`;
  const lim = (c) => (c.limit === "front" ? "Front (understeer)" : "Rear (oversteer)");
  const rows = [
    ["Entry speed", "km/h", (c) => kmh(c.v_entry), 1], ["Minimum speed", "km/h", (c) => kmh(c.v_apex), 1], ["Exit speed", "km/h", (c) => kmh(c.v_exit), 1],
    ["Peak cornering", "g", (c) => c.ay_peak, 2], ["Peak braking on approach", "g", (c) => c.brake_peak, 2], ["Peak acceleration on exit", "g", (c) => c.accel_exit, 2],
    ["Time in corner", "s", (c) => c.time, 2],
    ["Front tyre grip used at slowest point", "%", (c) => 100 * Math.min(c.uf, 1), 0], ["Rear tyre grip used at slowest point", "%", (c) => 100 * Math.min(c.ur, 1), 0],
    ["Body roll", "deg", (c) => Math.abs(c.roll), 2], ["Battery power at exit", "kW", (c) => c.p_exit, 1], ["Motor torque at exit", "N·m", (c) => c.torque_exit, 0],
  ];
  let html = `<p class="sub">Starts ${fmt(a.s_start, 0)} m into the lap, slowest point at ${fmt(a.s_apex, 0)} m. Axle nearest its grip limit: <b>${lim(a)}</b>${b ? `, B: <b>${lim(b)}</b>` : ""}.</p>`;
  html += `<table class="ctab"><thead><tr><th></th><th>${b ? '<i class="sw a"></i> A' : ""}</th>${b ? '<th><i class="sw b"></i> B</th><th>B − A</th>' : ""}</tr></thead><tbody>`;
  for (const [l, u, f, d] of rows) html += `<tr><td>${l}, ${u}</td><td>${fmt(f(a), d)}</td>${b ? `<td>${fmt(f(b), d)}</td><td class="delta">${signed(f(b) - f(a), d)}</td>` : ""}</tr>`;
  html += `</tbody></table><div class="loads">`;
  const tyres = (c, k) => {
    const mx = Math.max(...c.fz, 1), names = ["Front left", "Front right", "Rear left", "Rear right"];
    return `<div class="car"><div class="t">${b ? `<i class="sw ${k.toLowerCase()}"></i>` : ""}Tyre loads at slowest point${b ? ", " + k : ""}, N</div>` +
      c.fz.map((f, i) => `<div class="tyre ${k}"><div class="n">${names[i]}</div><div class="x">${f}</div><i style="width:${(100 * f) / mx}%"></i></div>`).join("") + `</div>`;
  };
  html += tyres(a, "A") + (b ? tyres(b, "B") : "") + `</div>`;
  host.innerHTML = html;
}
function drawTable() {
  const A = state.results.A, B = state.compare ? state.results.B : null, t = $("ctable");
  if (!A || !A.corners.length) { t.innerHTML = ""; return; }
  const head = ["#", "Turn", "Radius, m", "Entry, km/h", "Min, km/h", "Exit, km/h", "Cornering, g", "Braking, g", "Time, s"].concat(B ? ["B − A time, s"] : []);
  t.innerHTML = `<thead><tr>${head.map((h) => `<th>${h}</th>`).join("")}</tr></thead><tbody>` + A.corners.map((c, i) =>
    `<tr data-n="${c.n}" class="${c.n === state.corner ? "on" : ""}"><td>${c.n}</td><td>${c.dir}</td><td>${fmt(c.radius, 1)}</td><td>${fmt(kmh(c.v_entry), 1)}</td><td>${fmt(kmh(c.v_apex), 1)}</td><td>${fmt(kmh(c.v_exit), 1)}</td><td>${fmt(c.ay_peak, 2)}</td><td>${fmt(c.brake_peak, 2)}</td><td>${fmt(c.time, 2)}</td>${B && B.corners[i] ? `<td>${signed(B.corners[i].time - c.time, 2)}</td>` : ""}</tr>`).join("") + "</tbody>";
}

function renderAll() { drawKpis(); drawCorner(); drawTable(); drawMap(); drawCharts(); drawRide(); }

// ---------- running the simulation ----------
function schedule(delay = 300) { clearTimeout(runTimer); runTimer = setTimeout(runNow, delay); }
function runNow() {
  const want = state.compare ? ["A", "B"] : ["A"];
  const batch = ++reqId; pending = { batch, left: want.length, t0: performance.now() };
  const st = $("status"); st.textContent = "Simulating…"; st.classList.add("busy");
  for (const k of want) workers[k].postMessage({ id: `${batch}:${k}`, req: { track: state.trackId, car: state.setups[k].car, cond: state.setups[k].cond } });
}
function onWorker(e) {
  const m = e.data, st = $("status");
  if (m.type === "ready") {
    state.schema = m.schema;
    for (const t of m.tracks) state.tracks[t.id] = t;
    for (const k of ["A", "B"]) state.setups[k] = { car: { ...m.schema.car }, cond: { ...m.schema.cond } };
    buildTabs(m.tracks); buildForm();
    $("sources").innerHTML = m.tracks.map((t) => `<b>${t.name}:</b> ${t.source}`).join(" ");
    runNow();
  } else if (m.type === "result") {
    const [batch, k] = m.id.split(":");
    if (+batch !== pending.batch) return;
    state.results[k] = m.result;
    if (--pending.left === 0) {
      if (!state.compare) state.results.B = null;
      st.classList.remove("busy"); st.textContent = `Simulated in ${((performance.now() - pending.t0) / 1000).toFixed(2)} s`;
      renderAll();
    }
  } else { st.classList.remove("busy"); st.textContent = "Error: " + m.message; console.error(m.message); }
}

// ---------- form and tabs ----------
function buildTabs(tracks) {
  const nav = $("tracks");
  nav.innerHTML = tracks.map((t) => `<button data-id="${t.id}" class="${t.id === state.trackId ? "on" : ""}">${t.name}</button>`).join("");
  nav.onclick = (e) => {
    const b = e.target.closest("button"); if (!b) return;
    state.trackId = b.dataset.id; state.cursor = null; state.corner = 1; state.results = { A: null, B: null };
    nav.querySelectorAll("button").forEach((x) => x.classList.toggle("on", x === b));
    drawMap(); runNow();
  };
}
function buildForm() {
  const g = state.schema.groups, form = $("form"), groups = [{ ...g.cond, scope: "cond" }].concat(g.car.map((x) => ({ ...x, scope: "car" })));
  form.innerHTML = groups.map((grp, gi) => `<details ${gi < 2 ? "open" : ""}><summary>${grp.name}</summary>` + grp.params.map((p) =>
    `<div class="row"><label for="in_${p.key}" title="Click for an explanation">${p.label}</label>` +
    (p.key === "drive" ? `<select id="in_${p.key}" data-scope="${grp.scope}" data-key="${p.key}"><option>RWD</option><option>AWD</option><option>FWD</option></select>`
      : `<input id="in_${p.key}" type="number" data-scope="${grp.scope}" data-key="${p.key}" min="${p.min}" max="${p.max}" step="${p.step}"><span class="unit">${p.unit}</span>`) +
    `<div class="help">${p.help}</div></div>`).join("") + `</details>`).join("");
  form.addEventListener("click", (e) => { if (e.target.tagName === "LABEL") { e.preventDefault(); e.target.parentElement.classList.toggle("show"); } });
  form.addEventListener("input", (e) => {
    const el = e.target, key = el.dataset.key; if (!key) return;
    let v = el.tagName === "SELECT" ? el.value : parseFloat(el.value);
    if (el.tagName !== "SELECT") { if (isNaN(v)) return; v = Math.min(+el.max, Math.max(+el.min, v)); }
    state.setups[state.active][el.dataset.scope][key] = v;
    markChanged(el);
    if (state.active === "B" && !state.compare) { state.compare = true; $("compare").checked = true; }
    schedule();
  });
  form.addEventListener("submit", (e) => e.preventDefault());
  fillForm();
}
function markChanged(el) { el.classList.toggle("changed", String(state.setups[state.active][el.dataset.scope][el.dataset.key]) !== String(state.schema[el.dataset.scope][el.dataset.key])); }
function fillForm() {
  $("form").querySelectorAll("[data-key]").forEach((el) => { el.value = state.setups[state.active][el.dataset.scope][el.dataset.key]; markChanged(el); });
  $("tabA").classList.toggle("on", state.active === "A"); $("tabB").classList.toggle("on", state.active === "B");
}
function wire() {
  $("tabA").onclick = () => { state.active = "A"; fillForm(); renderAll(); };
  $("tabB").onclick = () => { state.active = "B"; fillForm(); if (!state.compare) { state.compare = true; $("compare").checked = true; runNow(); } else renderAll(); };
  $("compare").onchange = (e) => { state.compare = e.target.checked; if (!state.compare) { state.active = "A"; fillForm(); } runNow(); };
  $("copy").onclick = () => { state.setups.B = JSON.parse(JSON.stringify(state.setups.A)); fillForm(); if (state.compare) runNow(); };
  $("reset").onclick = () => { state.setups[state.active] = { car: { ...state.schema.car }, cond: { ...state.schema.cond } }; fillForm(); runNow(); };
  $("colour").onchange = (e) => { state.colour = e.target.value; drawMap(); };
  $("prevC").onclick = () => selectCorner(state.corner - 1);
  $("nextC").onclick = () => selectCorner(state.corner + 1);
  const map = $("map");
  map.addEventListener("mousemove", (e) => mapPointer(e, false));
  map.addEventListener("mouseleave", () => setCursor(null));
  map.addEventListener("click", (e) => mapPointer(e, true));
  $("ctable").addEventListener("click", (e) => { const tr = e.target.closest("tr[data-n]"); if (tr) selectCorner(+tr.dataset.n); });
  window.addEventListener("resize", () => { drawMap(); drawCharts(); drawRide(); });
}

wire();
// One engine per setup, so A and B are simulated side by side rather than in turn.
for (const k of ["A", "B"]) {
  workers[k] = new Worker(new URL("./worker.js", import.meta.url), { type: "module" });
  workers[k].onmessage = (e) => { if (e.data.type === "ready") { if (++ready === 2) onWorker(e); } else onWorker(e); };
  workers[k].onerror = (e) => { $("status").textContent = "Could not start the physics engine: " + (e.message || "worker error"); };
}
