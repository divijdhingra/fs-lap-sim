"""The lap solver.

Method (the standard quasi-steady-state lap time recipe):
  1. For every point on the track, find the highest speed the car could hold
     if the corner at that point went on forever (the "corner speed limit").
  2. Forward pass: drive along the track accelerating as hard as tyres and
     motor allow, never exceeding the corner speed limit.
  3. Backward pass: walk the track in reverse and lower any speed that the
     car could not have braked down from in time for the next corner.
  What is left is the fastest possible speed at every point. Time is distance
  divided by speed, added up.
"""
import math
from .vehicle import Vehicle, G, SOC_MIN


def _corner_limits(veh, ks):
    memo = {}
    out = []
    for k in ks:
        key = round(abs(k), 4)
        v = memo.get(key)
        if v is None:
            v = memo[key] = veh.corner_speed(key)
        out.append(v)
    return out


def _passes(veh, ks, vlim, ds, v_start, v_end=None):
    """Forward and backward passes over an open stretch of len(ks) points.
    Returns speeds at each point."""
    n = len(ks)
    v = list(vlim)
    v[0] = min(v[0], v_start)
    ax_max, ax_min = veh.ax_max, veh.ax_min
    two_ds = 2 * ds
    for i in range(n - 1):
        vi = v[i]
        a, _ = ax_max(max(vi, 0.3), ks[i])
        vn = math.sqrt(max(0.0, vi * vi + two_ds * a))
        if vn < v[i + 1]:
            v[i + 1] = vn
    if v_end is not None and v_end < v[-1]:
        v[-1] = v_end
    for i in range(n - 1, 0, -1):
        vi = v[i]
        if v[i - 1] <= vi:
            continue
        a = ax_min(vi, ks[i])
        vp = math.sqrt(vi * vi - two_ds * a)
        if vp < v[i - 1]:
            v[i - 1] = vp
    return v


def _solve_lap(veh, track, vlim, standing):
    """Speeds for one lap. Returns a list one longer than the track for closed
    tracks (the last entry is the start line again)."""
    ks, ds = track["k"], track["ds"]
    n = len(ks)
    if not track["closed"]:
        v0 = 0.0 if standing else min(track.get("entry_speed", 10.0), vlim[0])
        return _passes(veh, ks, vlim, ds, v0)
    # Closed lap. The slowest corner on the lap is always taken at its limit
    # speed, so start the passes there and go once round.
    j = min(range(n), key=vlim.__getitem__)
    rk = ks[j:] + ks[:j] + [ks[j]]
    rv = vlim[j:] + vlim[:j] + [vlim[j]]
    fly = _passes(veh, rk, rv, ds, rv[0], rv[0])
    fly = fly[n - j:n] + fly[:n - j]             # rotate back so index 0 = start line
    if not standing:
        return fly + [fly[0]]
    # Standing start: accelerate from rest, joining the flying lap once up to speed.
    kk, vv = ks + [ks[0]], fly + [fly[0]]
    return _passes(veh, kk, vv, ds, 0.0)


def _integrate(veh, track, v):
    """Turn a speed trace into time, accelerations, energy and display channels."""
    ks, ds = track["k"], track["ds"]
    nseg = len(v) - 1
    t = 0.0
    q_ah = 0.0; e_out = 0.0; e_regen = 0.0
    ch = dict(t=[], v=[], ax=[], ay=[], p=[], torque=[], uf=[], ur=[], roll=[],
              fz_fl=[], fz_fr=[], fz_rl=[], fz_rr=[])
    for i in range(nseg):
        v0, v1 = v[i], v[i + 1]
        vm = max(0.5 * (v0 + v1), 0.15)
        dt = ds / vm
        ax = (v1 * v1 - v0 * v0) / (2 * ds)
        k = ks[i % len(ks)]
        st = veh.point_state(vm, k, ax)
        p = st["p"]
        cur = veh.battery_current(p)
        q_ah += cur * dt / 3600.0
        if p >= 0: e_out += p * dt
        else: e_regen -= p * dt
        ch["t"].append(t); ch["v"].append(vm); ch["ax"].append(ax / G); ch["ay"].append(vm * vm * k / G)
        ch["p"].append(p / 1000.0); ch["torque"].append(st["torque"])
        ch["uf"].append(st["uf"]); ch["ur"].append(st["ur"]); ch["roll"].append(st["roll"])
        fz = st["fz"]
        ch["fz_fl"].append(fz[0]); ch["fz_fr"].append(fz[1]); ch["fz_rl"].append(fz[2]); ch["fz_rr"].append(fz[3])
        t += dt
    return dict(time=t, q_ah=q_ah, e_out_kwh=e_out / 3.6e6, e_regen_kwh=e_regen / 3.6e6, ch=ch)


def _find_corners(track, ch):
    """Split the lap into numbered corners from the curvature trace."""
    ks, ds = track["k"], track["ds"]
    rad = track.get("corner_radius", 0)
    n = len(ch["v"])
    if not rad:
        return []
    thr = 1.0 / rad
    gap = max(2, int((6.0 if rad < 100 else 40.0) / ds))
    minlen = max(2, int((4.0 if rad < 100 else 25.0) / ds))
    runs = []
    i = 0
    while i < n:
        if abs(ks[i]) > thr:
            sgn = ks[i] > 0
            j = i
            while j + 1 < n and abs(ks[j + 1]) > thr and (ks[j + 1] > 0) == sgn:
                j += 1
            runs.append([i, j, sgn]); i = j + 1
        else:
            i += 1
    merged = []
    for r in runs:
        if merged and merged[-1][2] == r[2] and r[0] - merged[-1][1] <= gap:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    corners = []
    v, ax, ay = ch["v"], ch["ax"], ch["ay"]
    for a, b, left in merged:
        if b - a < minlen:
            continue
        apex = min(range(a, b + 1), key=v.__getitem__)
        prev_end = corners[-1]["end_i"] if corners else max(0, a - int(150 / ds))
        lo = max(prev_end, a - int(150 / ds))
        brake = min(ax[lo:apex + 1]) if apex >= lo else 0.0
        kmax = max(abs(ks[q]) for q in range(a, b + 1))
        corners.append(dict(
            n=len(corners) + 1, dir="left" if left else "right",
            start_i=a, apex_i=apex, end_i=b,
            s_start=a * ds, s_apex=apex * ds, s_end=b * ds,
            radius=1.0 / kmax,
            v_entry=v[a], v_apex=v[apex], v_exit=v[b],
            ay_peak=max(abs(ay[q]) for q in range(a, b + 1)),
            brake_peak=-min(0.0, brake),
            accel_exit=max(ax[apex:b + 1]),
            time=ch["t"][b] - ch["t"][a] + ds / max(v[b], 0.15),
            limit="front" if ch["uf"][apex] >= ch["ur"][apex] else "rear",
            uf=ch["uf"][apex], ur=ch["ur"][apex],
            fz=[ch["fz_fl"][apex], ch["fz_fr"][apex], ch["fz_rl"][apex], ch["fz_rr"][apex]],
            p_exit=ch["p"][b], torque_exit=ch["torque"][b], roll=ch["roll"][apex],
        ))
    return corners


def _time_at(ch, ds, s):
    i = min(len(ch["t"]) - 1, int(s / ds))
    return ch["t"][i] + (s - i * ds) / max(ch["v"][i], 0.15)


def _round_ch(ch):
    dec = dict(t=3, v=2, ax=3, ay=3, p=2, torque=1, uf=3, ur=3, roll=2)
    return {k: [round(x, dec.get(k, 0)) for x in a] for k, a in ch.items()}


def simulate(car, cond, track):
    """Run one event. Returns a plain dict ready to be turned into JSON."""
    veh = Vehicle(car, cond)
    track = dict(track)
    track["k"] = list(track["k"])
    ds = track["ds"]
    laps = track.get("laps", 1)
    standing = track.get("standing_start", False)
    soc0 = veh.soc
    wear0 = veh.wear
    lap_km = len(track["k"]) * ds / 1000.0
    lap_rows = []
    total = 0.0; e_out = 0.0; e_regen = 0.0
    shown = None
    finished = True
    vlim = None; vlim_wear = None
    for lap in range(1, laps + 1):
        if veh.soc <= SOC_MIN:
            finished = False
            break
        # Corner speed limits only change with tyre wear; refresh every 2 % of wear.
        if vlim is None or abs(veh.wear - vlim_wear) >= 2.0:
            vlim = _corner_limits(veh, track["k"]); vlim_wear = veh.wear
        v = _solve_lap(veh, track, vlim, standing and lap == 1)
        res = _integrate(veh, track, v)
        p_avail = veh.p_elec / 1000.0
        soc_end = veh.soc - res["q_ah"] / veh.cap_ah
        total += res["time"]; e_out += res["e_out_kwh"]; e_regen += res["e_regen_kwh"]
        lap_rows.append(dict(lap=lap, time=round(res["time"], 3), soc_end=round(soc_end * 100, 2),
                             e_kwh=round(res["e_out_kwh"] - res["e_regen_kwh"], 4),
                             p_avail=round(p_avail, 1), wear=round(veh.wear, 2)))
        # Show the first flying lap for multi-lap events, otherwise the only lap.
        if shown is None and (laps == 1 or lap == 2):
            shown = res; shown_lap = lap
        veh.set_state(soc_end, veh.wear + cond["wear_rate"] * lap_km if laps > 1 else veh.wear)
    if shown is None:
        shown = res; shown_lap = 1
    ch = shown["ch"]
    npt = len(ch["v"])
    out = dict(
        track_id=track["id"], kind=track["kind"], laps=laps, ds=ds, shown_lap=shown_lap,
        time=round(total, 3), lap_time=round(shown["time"], 3), finished=finished,
        laps_done=len(lap_rows),
        energy_kwh=round(e_out - e_regen, 4), regen_kwh=round(e_regen, 4),
        soc_start=round(soc0 * 100, 2), soc_end=lap_rows[-1]["soc_end"] if lap_rows else round(soc0 * 100, 2),
        v_max=round(max(ch["v"]), 2), v_min=round(min(ch["v"]), 2),
        ay_max=round(max(abs(a) for a in ch["ay"]), 3),
        ax_max=round(max(ch["ax"]), 3), ax_min=round(min(ch["ax"]), 3),
        p_max=round(max(ch["p"]), 1),
        mass=veh.m, v_top=round(veh.v_top, 2), p_avail=round(Vehicle(car, cond).p_elec / 1000.0, 1),
        lap_rows=lap_rows if laps > 1 else [],
        corners=_find_corners(track, ch),
        ch=_round_ch(ch),
    )
    if track["kind"] == "skidpad":
        ts = [_time_at(ch, ds, b) - _time_at(ch, ds, a) for a, b in track["timed"]]
        out["time"] = round(sum(ts) / len(ts), 3)
        out["skidpad_laps"] = [round(x, 3) for x in ts]
    for c in out["corners"]:
        for k, val in c.items():
            if isinstance(val, float):
                c[k] = round(val, 3)
        c["fz"] = [round(x) for x in c["fz"]]
    return out
