// Track builder: draw the middle of a coned lane, see the cones it implies, save it.
// A layout is { name, width (lane width, m), pts: [[x, y], ...] } with x, y in metres.
// pts are the points the user placed, in driving order; the first is the start line.

const $ = (id) => document.getElementById(id);
export const LKEY = "fslap.layouts.v1";
export const CONE = { left: "#4aa3ff", right: "#f2c84b", orange: "#f08a3c" };
const CONE_GAP = 4.0;            // m between cones along each side (the rules allow up to 5 m)
const W_MIN = 3.0, W_MAX = 6.0;  // m, lane width range (the rules ask for at least 3 m)
const LEN_MIN = 30, LEN_MAX = 3000;
const EXAMPLE = [[0, 0], [28, 1], [52, -4], [62, -22], [50, -38], [30, -30], [12, -44], [-12, -40], [-24, -20], [-16, -4]];

// ---------- storage ----------
export function cleanLayout(o) {
  if (!o || typeof o !== "object" || !Array.isArray(o.pts)) return null;
  const pts = o.pts.filter((p) => Array.isArray(p) && p.length >= 2 && Number.isFinite(+p[0]) && Number.isFinite(+p[1]) && Math.abs(p[0]) < 5000 && Math.abs(p[1]) < 5000)
    .slice(0, 300).map((p) => [Math.round(p[0] * 100) / 100, Math.round(p[1] * 100) / 100]);
  if (pts.length < 4) return null;
  const width = Math.min(W_MAX, Math.max(W_MIN, +o.width || 3.5));
  const name = String(o.name || "My layout").trim().slice(0, 40) || "My layout";
  return { name, width: Math.round(width * 10) / 10, pts };
}
export function readLayouts() {
  try { const v = JSON.parse(localStorage.getItem(LKEY)); return Array.isArray(v) ? v.map(cleanLayout).filter(Boolean) : []; } catch { return []; }
}
export function writeLayouts(a) { try { localStorage.setItem(LKEY, JSON.stringify(a)); return true; } catch { return false; } }
export function layoutId(l) {
  const s = JSON.stringify([l.pts, l.width]); let h = 2166136261;
  for (let i = 0; i < s.length; i++) { h ^= s.charCodeAt(i); h = Math.imul(h, 16777619); }
  return "my_" + (h >>> 0).toString(36);
}

// ---------- geometry ----------
// A smooth closed curve through the placed points (centripetal Catmull-Rom spline:
// it passes through every point and does not loop or overshoot between them).
function spline(pts) {
  const n = pts.length, out = [], seg = [];
  const mix = (a, b, ta, tb, t) => { const f = (t - ta) / (tb - ta); return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]; };
  for (let i = 0; i < n; i++) {
    const p0 = pts[(i - 1 + n) % n], p1 = pts[i], p2 = pts[(i + 1) % n], p3 = pts[(i + 2) % n];
    const k = (a, b) => Math.max(1e-3, Math.sqrt(Math.hypot(b[0] - a[0], b[1] - a[1])));
    const t0 = 0, t1 = k(p0, p1), t2 = t1 + k(p1, p2), t3 = t2 + k(p2, p3);
    const steps = Math.max(4, Math.ceil(Math.hypot(p2[0] - p1[0], p2[1] - p1[1]) / 0.5));
    for (let j = 0; j < steps; j++) {
      const t = t1 + (t2 - t1) * (j / steps);
      const a1 = mix(p0, p1, t0, t1, t), a2 = mix(p1, p2, t1, t2, t), a3 = mix(p2, p3, t2, t3, t);
      const b1 = mix(a1, a2, t0, t2, t), b2 = mix(a2, a3, t1, t3, t);
      out.push(mix(b1, b2, t1, t2, t)); seg.push(i);
    }
  }
  return { out, seg };
}
function resample(P, step) {
  const n = P.length, s = [0];
  for (let i = 0; i < n; i++) { const q = P[(i + 1) % n]; s.push(s[i] + Math.hypot(q[0] - P[i][0], q[1] - P[i][1])); }
  const total = s[n], m = Math.max(8, Math.round(total / step)), d = total / m, out = [];
  let i = 0;
  for (let q = 0; q < m; q++) {
    const t = q * d; while (s[i + 1] < t) i++;
    const f = s[i + 1] > s[i] ? (t - s[i]) / (s[i + 1] - s[i]) : 0, b = P[(i + 1) % n];
    out.push([P[i][0] + f * (b[0] - P[i][0]), P[i][1] + f * (b[1] - P[i][1])]);
  }
  return { out, d, total };
}
const cross = (a, b, c, d) => {            // do segments ab and cd cross?
  const o = (p, q, r) => (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
  return o(a, b, c) * o(a, b, d) < 0 && o(c, d, a) * o(c, d, b) < 0;
};

// Everything the page needs to know about a layout: the lane middle, the cones, and whether it is usable.
export function geometry(l) {
  const g = { ok: false, problems: [], warnings: [], centre: [], cones: { left: [], right: [], orange: [] }, length: 0, minR: Infinity, curve: [], seg: [] };
  if (l.pts.length < 4) { g.problems.push(`Place at least 4 points (${l.pts.length} so far).`); if (l.pts.length < 2) return g; }
  const sp = spline(l.pts.length >= 3 ? l.pts : [...l.pts, l.pts[0]]);
  g.curve = sp.out; g.seg = sp.seg;
  const { out: C, d, total } = resample(sp.out, 1.0);
  g.centre = C; g.length = total;
  const n = C.length, hw = l.width / 2, nor = [];
  for (let i = 0; i < n; i++) {
    const a = C[(i - 1 + n) % n], b = C[(i + 1) % n], L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
    nor.push([-(b[1] - a[1]) / L, (b[0] - a[0]) / L]);                     // points to the left of travel
    // corner radius from the circle through three points 2 m apart
    const p = C[(i - 2 + n) % n], q = C[i], r = C[(i + 2) % n];
    const A = Math.hypot(q[0] - p[0], q[1] - p[1]), B = Math.hypot(r[0] - q[0], r[1] - q[1]), Cc = Math.hypot(r[0] - p[0], r[1] - p[1]);
    const area = Math.abs((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])) / 2;
    if (area > 1e-6) g.minR = Math.min(g.minR, (A * B * Cc) / (4 * area));
  }
  // cones: blue on the left, yellow on the right, evenly spaced; none where they would sit inside the lane
  const nc = Math.max(4, Math.round(total / CONE_GAP));
  const clear = (p) => { for (let j = 0; j < n; j++) if (Math.hypot(C[j][0] - p[0], C[j][1] - p[1]) < hw - 0.25) return false; return true; };
  for (let c = 0; c < nc; c++) {
    const i = Math.min(n - 1, Math.round(((c + 0.5) * n) / nc));
    const lp = [C[i][0] + nor[i][0] * hw, C[i][1] + nor[i][1] * hw], rp = [C[i][0] - nor[i][0] * hw, C[i][1] - nor[i][1] * hw];
    if (clear(lp)) g.cones.left.push(lp.map((v) => Math.round(v * 100) / 100));
    if (clear(rp)) g.cones.right.push(rp.map((v) => Math.round(v * 100) / 100));
  }
  for (const side of [1, -1]) for (const along of [-0.3, 0.3])           // four big orange cones mark the start line
    g.cones.orange.push([C[0][0] + nor[0][0] * side * (hw + 0.4) - nor[0][1] * along, C[0][1] + nor[0][1] * side * (hw + 0.4) + nor[0][0] * along].map((v) => Math.round(v * 100) / 100));
  // checks
  const stepc = Math.max(1, Math.ceil(n / 700)); let crossing = false, touching = false;
  const far = Math.ceil((2 * l.width + 4) / d);
  for (let i = 0; i < n && !crossing; i += stepc) for (let j = i + 2 * stepc; j < n; j += stepc) {
    const sep = Math.min(j - i, n - (j - i));
    if (sep <= stepc) continue;
    if (cross(C[i], C[(i + stepc) % n], C[j], C[(j + stepc) % n])) { crossing = true; break; }
    if (sep > far && Math.hypot(C[i][0] - C[j][0], C[i][1] - C[j][1]) < l.width) touching = true;
  }
  if (crossing) g.problems.push("The layout crosses over itself. Move a point so the loop does not cross.");
  else if (touching) g.problems.push("Two parts of the lane overlap. Move them at least one lane width apart.");
  if (total < LEN_MIN) g.problems.push(`The loop is ${total.toFixed(0)} m long. It needs to be at least ${LEN_MIN} m.`);
  if (total > LEN_MAX) g.problems.push(`The loop is ${total.toFixed(0)} m long. Keep it under ${LEN_MAX} m.`);
  if (g.minR < hw + 0.5) g.problems.push(`The tightest corner (${g.minR.toFixed(1)} m radius) is tighter than the lane is wide, so the cones would fold over each other. Open it out or narrow the lane.`);
  else if (g.minR + hw < 4.5) g.warnings.push(`Tightest corner: ${(2 * (g.minR + hw)).toFixed(1)} m outside diameter. The rules ask for at least 9 m at hairpins.`);
  g.ok = g.problems.length === 0;
  return g;
}

// ---------- the editor ----------
let ed = null;
export function openBuilder(layout, taken, onSave) {
  ed = {
    l: layout ? JSON.parse(JSON.stringify(layout)) : { name: "", width: 3.5, pts: [] },
    original: layout ? layout.name : null, taken, onSave, undo: [], view: { cx: 20, cy: -20, s: 6 }, drag: null, g: null,
  };
  $("bName").value = ed.l.name; $("bWidth").value = ed.l.width;
  $("builder").hidden = false; document.body.classList.add("modal-open");
  $("bTitle").textContent = layout ? "Edit layout" : "Draw a new layout";
  fit(); refresh();
}
function close() { $("builder").hidden = true; document.body.classList.remove("modal-open"); ed = null; }
function push() { ed.undo.push(JSON.stringify(ed.l.pts)); if (ed.undo.length > 60) ed.undo.shift(); }
function fit() {
  const cv = $("bCanvas"), w = cv.clientWidth || 800, h = cv.clientHeight || 560, p = ed.l.pts;
  if (p.length < 2) { ed.view = { cx: 20, cy: -20, s: Math.min(w / 120, h / 80) }; return; }
  const xs = p.map((q) => q[0]), ys = p.map((q) => q[1]);
  const x0 = Math.min(...xs) - 12, x1 = Math.max(...xs) + 12, y0 = Math.min(...ys) - 12, y1 = Math.max(...ys) + 12;
  ed.view = { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, s: Math.min(w / (x1 - x0), h / (y1 - y0)) };
}
const toPx = (p) => { const cv = $("bCanvas"); return [cv.clientWidth / 2 + (p[0] - ed.view.cx) * ed.view.s, cv.clientHeight / 2 - (p[1] - ed.view.cy) * ed.view.s]; };
const toM = (x, y) => { const cv = $("bCanvas"); return [ed.view.cx + (x - cv.clientWidth / 2) / ed.view.s, ed.view.cy - (y - cv.clientHeight / 2) / ed.view.s]; };

function refresh() {
  ed.l.width = Math.min(W_MAX, Math.max(W_MIN, parseFloat($("bWidth").value) || 3.5));
  ed.g = geometry(ed.l);
  draw();
  const g = ed.g, n = ed.l.pts.length;
  $("bStats").innerHTML = n < 2 ? `<div class="empty">Click on the grid to place the first point. That point is the start line.</div>` :
    [["Lap length", g.length.toFixed(0) + " m"], ["Endurance (22 km)", Math.ceil(22000 / Math.max(g.length, 1)) + " laps"],
     ["Tightest corner", isFinite(g.minR) && g.minR < 500 ? g.minR.toFixed(1) + " m radius" : "none"],
     ["Cones", g.cones.left.length + g.cones.right.length + 4], ["Points placed", n]].map(([a, b]) => `<div><span>${a}</span><b>${b}</b></div>`).join("");
  $("bMsgs").innerHTML = g.problems.map((m) => `<p class="bad">${m}</p>`).join("") + g.warnings.map((m) => `<p class="warn">${m}</p>`).join("");
  $("bSave").disabled = !g.ok; $("bUndo").disabled = !ed.undo.length; $("bExport").disabled = !g.ok;
}

function draw() {
  const cv = $("bCanvas"), dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const ctx = cv.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
  const { s } = ed.view, [mx0, my1] = toM(0, 0), [mx1, my0] = toM(w, h);
  // grid: a line every 10 m, labelled, so distances can be read off
  const step = s > 2.2 ? 10 : s > 0.9 ? 50 : 100;
  ctx.lineWidth = 1; ctx.font = "10px Inter, system-ui, sans-serif"; ctx.fillStyle = "#6c716a";
  for (let x = Math.ceil(mx0 / step) * step; x <= mx1; x += step) { const px = toPx([x, 0])[0]; ctx.strokeStyle = x === 0 ? "#3a3d3a" : "#222422"; ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke(); ctx.fillText(x + " m", px + 3, h - 5); }
  for (let y = Math.ceil(my0 / step) * step; y <= my1; y += step) { const py = toPx([0, y])[1]; ctx.strokeStyle = y === 0 ? "#3a3d3a" : "#222422"; ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke(); ctx.fillText(y + " m", 4, py - 3); }
  const g = ed.g, P = g.curve.map(toPx);
  if (P.length > 1) {
    const path = () => { ctx.beginPath(); P.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); if (ed.l.pts.length >= 3) ctx.closePath(); };
    ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.strokeStyle = g.ok ? "#2c2f2c" : "#3a2622"; ctx.lineWidth = Math.max(2, ed.l.width * s); path(); ctx.stroke();      // the lane
    ctx.strokeStyle = "#7d837b"; ctx.lineWidth = 1; ctx.setLineDash([5, 5]); path(); ctx.stroke(); ctx.setLineDash([]);     // its middle
    const r = Math.max(1.8, 0.16 * s);
    for (const k of ["left", "right", "orange"]) { ctx.fillStyle = CONE[k]; for (const c of g.cones[k]) { const p = toPx(c); ctx.beginPath(); ctx.arc(p[0], p[1], k === "orange" ? r * 1.4 : r, 0, 7); ctx.fill(); } }
  }
  ed.l.pts.forEach((q, i) => {
    const p = toPx(q);
    ctx.beginPath(); ctx.arc(p[0], p[1], i === 0 ? 7 : 5.5, 0, 7); ctx.fillStyle = i === 0 ? "#3fb37f" : "#eceee9"; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = "#111312"; ctx.stroke();
  });
  if (ed.l.pts.length) {                      // start label and an arrow for the driving direction
    const p = toPx(ed.l.pts[0]); ctx.fillStyle = "#eceee9"; ctx.font = "600 11px Inter, system-ui, sans-serif"; ctx.fillText("Start", p[0] + 11, p[1] - 10);
    if (P.length > 12) {
      const a = P[4], b = P[12], ang = Math.atan2(b[1] - a[1], b[0] - a[0]);
      ctx.save(); ctx.translate(b[0], b[1]); ctx.rotate(ang); ctx.fillStyle = "#3fb37f"; ctx.beginPath(); ctx.moveTo(7, 0); ctx.lineTo(-5, -5); ctx.lineTo(-5, 5); ctx.closePath(); ctx.fill(); ctx.restore();
    }
  }
}

function wire() {
  const cv = $("bCanvas"), pos = (e) => { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
  const hitPoint = (x, y) => ed.l.pts.findIndex((q) => { const p = toPx(q); return Math.hypot(p[0] - x, p[1] - y) < 10; });
  const remove = (i) => { if (i >= 0) { push(); ed.l.pts.splice(i, 1); refresh(); } };
  cv.addEventListener("pointerdown", (e) => {
    if (!ed || e.button !== 0) return;
    const [x, y] = pos(e), i = hitPoint(x, y);
    cv.setPointerCapture(e.pointerId);
    ed.drag = i >= 0 ? { i, moved: false, x, y } : e.shiftKey ? { pan: true, x, y, cx: ed.view.cx, cy: ed.view.cy } : { add: true, x, y };
  });
  cv.addEventListener("pointermove", (e) => {
    if (!ed) return;
    const [x, y] = pos(e), d = ed.drag;
    if (!d) { cv.style.cursor = hitPoint(x, y) >= 0 ? "grab" : "crosshair"; return; }
    if (d.pan) { ed.view.cx = d.cx - (x - d.x) / ed.view.s; ed.view.cy = d.cy + (y - d.y) / ed.view.s; draw(); }
    else if (d.i != null) {
      if (!d.moved && Math.hypot(x - d.x, y - d.y) < 3) return;
      if (!d.moved) { push(); d.moved = true; }
      ed.l.pts[d.i] = toM(x, y).map((v) => Math.round(v * 10) / 10); refresh();
    }
  });
  cv.addEventListener("pointerup", (e) => {
    if (!ed || !ed.drag) return;
    const [x, y] = pos(e), d = ed.drag; ed.drag = null;
    if (!d.add || Math.hypot(x - d.x, y - d.y) > 4) return;
    const m = toM(x, y).map((v) => Math.round(v * 10) / 10);
    // a click on the line adds a point there; a click anywhere else adds the next point in driving order
    let at = ed.l.pts.length, best = 12;
    ed.g.curve.forEach((c, j) => { const p = toPx(c), dd = Math.hypot(p[0] - x, p[1] - y); if (dd < best) { best = dd; at = ed.g.seg[j] + 1; } });
    push(); ed.l.pts.splice(at, 0, m); refresh();
  });
  cv.addEventListener("dblclick", (e) => { if (ed) remove(hitPoint(...pos(e))); });
  cv.addEventListener("contextmenu", (e) => { e.preventDefault(); if (ed) remove(hitPoint(...pos(e))); });
  cv.addEventListener("wheel", (e) => {
    if (!ed) return; e.preventDefault();
    const [x, y] = pos(e), before = toM(x, y);
    ed.view.s = Math.min(40, Math.max(0.25, ed.view.s * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
    const after = toM(x, y); ed.view.cx += before[0] - after[0]; ed.view.cy += before[1] - after[1]; draw();
  }, { passive: false });
  const zoom = (f) => { ed.view.s = Math.min(40, Math.max(0.25, ed.view.s * f)); draw(); };
  $("bZoomIn").onclick = () => zoom(1.3); $("bZoomOut").onclick = () => zoom(1 / 1.3); $("bFit").onclick = () => { fit(); draw(); };
  $("bWidth").oninput = refresh;
  $("bUndo").onclick = () => { if (ed.undo.length) { ed.l.pts = JSON.parse(ed.undo.pop()); refresh(); } };
  $("bReverse").onclick = () => { if (ed.l.pts.length > 1) { push(); ed.l.pts = [ed.l.pts[0], ...ed.l.pts.slice(1).reverse()]; refresh(); } };
  $("bClear").onclick = () => { if (ed.l.pts.length) { push(); ed.l.pts = []; refresh(); } };
  $("bExample").onclick = () => { push(); ed.l.pts = EXAMPLE.map((p) => [...p]); fit(); refresh(); };
  $("bCancel").onclick = $("bClose").onclick = close;
  $("builder").addEventListener("mousedown", (e) => { if (e.target === $("builder")) close(); });
  window.addEventListener("keydown", (e) => { if (ed && e.key === "Escape") close(); });
  window.addEventListener("resize", () => { if (ed) draw(); });
  $("bExport").onclick = () => {
    const l = cleanLayout({ ...ed.l, name: $("bName").value }); if (!l) return;
    const blob = new Blob([JSON.stringify({ app: "fs-lap-sim", type: "layout", version: 1, ...l }, null, 1)], { type: "application/json" });
    const a = document.createElement("a"); a.href = URL.createObjectURL(blob);
    a.download = (l.name.replace(/[^a-z0-9]+/gi, "-").replace(/^-|-$/g, "").toLowerCase() || "layout") + ".fstrack.json";
    a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  };
  $("bImport").onclick = () => $("bFile").click();
  $("bFile").onchange = async (e) => {
    const f = e.target.files[0]; e.target.value = ""; if (!f) return;
    let l = null; try { l = cleanLayout(JSON.parse(await f.text())); } catch { /* not JSON */ }
    if (!l) { $("bMsgs").innerHTML = `<p class="bad">That file is not a layout saved from this page.</p>`; return; }
    push(); ed.l.pts = l.pts; $("bWidth").value = l.width; if (!$("bName").value) $("bName").value = l.name; fit(); refresh();
  };
  $("bSave").onclick = () => {
    let name = $("bName").value.trim().slice(0, 40);
    if (!name) { let k = 1; while (ed.taken.includes(`My layout ${k}`)) k++; name = `My layout ${k}`; }
    if (name !== ed.original && ed.taken.includes(name)) { $("bMsgs").innerHTML = `<p class="bad">You already have a layout called "${name.replace(/[<>&"]/g, "")}". Choose another name.</p>`; return; }
    const l = cleanLayout({ ...ed.l, name }), cb = ed.onSave, orig = ed.original;
    close(); cb(l, orig);
  };
}
wire();
