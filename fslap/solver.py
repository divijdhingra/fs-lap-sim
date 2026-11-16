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
from .ride import ride_replay, bump_metrics
from .tyre import temp_step, temp_grip, wear_added
from .thermal import heat_step, MOTOR_HEAT_SHARE, MOTOR_THERMAL_MASS


def _corner_limits(veh, ks, offs=None, base=None):
    """Corner speed limit at every track point. offs: optional per-point tyre
    load offsets from the spring and damper replay; base: the limits without
    offsets, used as starting guesses."""
    if offs is None:
        memo = {}
        out = []
        for k in ks:
            key = round(abs(k), 4)
            v = memo.get(key)
            if v is None:
                v = memo[key] = veh.corner_speed(key)
            out.append(v)
        return out
    return [veh.corner_speed(k, offs[i], base[i]) if abs(k) > 1e-4 else base[i] for i, k in enumerate(ks)]


def _passes(veh, ks, vlim, ds, v_start, v_end=None, offs=None, h0=0):
    """Forward and backward passes over an open stretch of len(ks) points.
    Returns speeds at each point. h0: track index of the first point, so the
    answers from the previous solve at the same place can be used as guesses."""
    n = len(ks)
    ha, hb = veh.hint_a, veh.hint_b
    hn = len(ha) if ha is not None else 0
    v = list(vlim)
    v[0] = min(v[0], v_start)
    ax_max, ax_min = veh.ax_max, veh.ax_min
    if offs is None:
        offs = [None] * n
    two_ds = 2 * ds
    for i in range(n - 1):
        vi = v[i]
        if hn:
            q = (h0 + i) % hn
            a, _ = ax_max(max(vi, 0.3), ks[i], offs[i], ha[q])
            ha[q] = a
        else:
            a, _ = ax_max(max(vi, 0.3), ks[i], offs[i])
        vn = math.sqrt(max(0.0, vi * vi + two_ds * a))
        if vn < v[i + 1]:
            v[i + 1] = vn
    if v_end is not None and v_end < v[-1]:
        v[-1] = v_end
    for i in range(n - 1, 0, -1):
        vi = v[i]
        if v[i - 1] <= vi:
            continue
        if hn:
            q = (h0 + i) % hn
            a = ax_min(vi, ks[i], offs[i], hb[q])
            hb[q] = a
        else:
            a = ax_min(vi, ks[i], offs[i])
        vp = math.sqrt(vi * vi - two_ds * a)
        if vp < v[i - 1]:
            v[i - 1] = vp
    if veh.lift_m > 0:
        _lift_and_coast(veh, ks, v, ds, offs, h0)
    return v


LIFT_FULL_DROP = 3.0     # m/s: speed lost in a braking zone at or above which the full lift distance is used


def _lift_and_coast(veh, ks, v, ds, offs, h0=0):
    """Driver lifts off the throttle a set distance before each braking point
    and lets the car roll (no drive, no brakes) until the braking line is met.
    Saves energy in endurance at a small cost in lap time. The car is never
    allowed to roll down below the speed the corner ahead will be taken at.

    v holds the fastest possible speeds. Mark the stretch from the lift point
    to the end of each braking zone as "no drive", then drive forward again
    with the old speeds as the ceiling."""
    n = len(v)
    ha = veh.hint_a
    hn = len(ha) if ha is not None else 0
    w = max(1, int(round(veh.lift_m / ds)))
    m, two_ds = veh.m, 2 * ds
    coast = [None] * n          # for no-drive points: the speed at the end of that braking zone
    b = 1
    while b < n - 1:
        if v[b] >= v[b - 1] and v[b + 1] < v[b]:
            e = b
            while e < n - 1 and v[e + 1] < v[e]:
                e += 1
            # Lift the full distance before a real braking zone, and in proportion
            # before a small dab of the brakes, so a tiny change in the lap cannot
            # switch a whole lift on or off.
            wz = int(round(w * min(1.0, (v[b] - v[e]) / LIFT_FULL_DROP)))
            # real braking, not just drag or easing round a tightening bend: judged on
            # the average slowing over the whole zone
            hard = (v[b] * v[b] - v[e] * v[e]) / (two_ds * (e - b)) > 2 * veh.resist(v[b]) / m + 1.0
            if wz >= 1 and hard:
                for i in range(max(0, b - wz), e):
                    coast[i] = v[e]
            b = e
        b += 1
    for i in range(n - 1):
        vi = v[i]
        if coast[i] is not None and vi > coast[i]:
            # rolling, but never slower than the corner ahead will be taken
            vn = max(coast[i], math.sqrt(max(0.0, vi * vi - two_ds * veh.resist(vi) / m)))
        else:
            a, _ = veh.ax_max(max(vi, 0.3), ks[i], offs[i], ha[(h0 + i) % hn] if hn else None)
            vn = math.sqrt(max(0.0, vi * vi + two_ds * a))
        if vn < v[i + 1]:
            v[i + 1] = vn


def _solve_lap(veh, track, vlim, standing, offs=None):
    """Speeds for one lap. Returns a list one longer than the track for closed
    tracks (the last entry is the start line again)."""
    ks, ds = track["k"], track["ds"]
    n = len(ks)
    if not track["closed"]:
        v0 = 0.0 if standing else min(track.get("entry_speed", 10.0), vlim[0])
        return _passes(veh, ks, vlim, ds, v0, None, offs)
    # Closed lap. The slowest corner on the lap is always taken at its limit
    # speed, so start the passes there and go once round.
    j = min(range(n), key=vlim.__getitem__)
    rk = ks[j:] + ks[:j] + [ks[j]]
    rv = vlim[j:] + vlim[:j] + [vlim[j]]
    ro = None if offs is None else offs[j:] + offs[:j] + [offs[j]]
    fly = _passes(veh, rk, rv, ds, rv[0], rv[0], ro, j)
    fly = fly[n - j:n] + fly[:n - j]             # rotate back so index 0 = start line
    if not standing:
        return fly + [fly[0]]
    # Standing start: accelerate from rest, joining the flying lap once up to speed.
    kk, vv = ks + [ks[0]], fly + [fly[0]]
    return _passes(veh, kk, vv, ds, 0.0, None, None if offs is None else offs + [offs[0]])


def _integrate(veh, track, v, offs=None):
    """Turn a speed trace into time, accelerations, energy and display channels."""
    ks, ds = track["k"], track["ds"]
    nseg = len(v) - 1
    t = 0.0
    q_ah = 0.0; e_out = 0.0; e_regen = 0.0
    ch = dict(t=[], v=[], ax=[], ay=[], p=[], torque=[], uf=[], ur=[], roll=[],
              fz_fl=[], fz_fr=[], fz_rl=[], fz_rr=[], slip_f=[], slip_r=[], steer=[], slip_ratio=[], temp_f=[], temp_r=[], cam_fo=[], cam_ro=[], batt_temp=[], motor_temp=[], tv=[])
    tf, tr, air = veh.temp_f, veh.temp_r, veh.air_temp      # tyre temperatures at the start of this lap
    tf_sum = tr_sum = 0.0; sl_f = sl_r = 0.0
    tb, tm = veh.batt_temp, veh.motor_temp                  # pack and motor temperatures at the start of this lap
    for i in range(nseg):
        v0, v1 = v[i], v[i + 1]
        vm = max(0.5 * (v0 + v1), 0.15)
        dt = ds / vm
        ax = (v1 * v1 - v0 * v0) / (2 * ds)
        k = ks[i % len(ks)]
        st = veh.point_state(vm, k, ax, offs[i % len(ks)] if offs is not None else None)
        p = st["p"]
        cur = veh.battery_current(p)
        q_ah += cur * dt / 3600.0
        if p >= 0: e_out += p * dt
        else: e_regen -= p * dt
        ch["t"].append(t); ch["v"].append(vm); ch["ax"].append(ax / G); ch["ay"].append(vm * vm * k / G)
        ch["p"].append(p / 1000.0); ch["torque"].append(st["torque"])
        ch["uf"].append(st["uf"]); ch["ur"].append(st["ur"]); ch["roll"].append(st["roll"])
        ch["slip_f"].append(st["slip_f"]); ch["slip_r"].append(st["slip_r"])
        ch["steer"].append(st["steer"]); ch["slip_ratio"].append(st["slip_ratio"])
        ch["cam_fo"].append(st["cam_fo"]); ch["cam_ro"].append(st["cam_ro"]); ch["tv"].append(st["tv"])
        fz = st["fz"]
        ch["fz_fl"].append(fz[0]); ch["fz_fr"].append(fz[1]); ch["fz_rl"].append(fz[2]); ch["fz_rr"].append(fz[3])
        # tyre temperature: heated by sliding and rolling losses, cooled by the air
        tf = temp_step(tf, st["heat_f"], vm, air, dt); tr = temp_step(tr, st["heat_r"], vm, air, dt)
        ch["temp_f"].append(tf); ch["temp_r"].append(tr)
        # pack heated by current through its internal resistance; motor by its share of drivetrain losses
        tb = heat_step(tb, cur * cur * veh.r_int, veh.batt_cool, veh.batt_mass_jk, air, dt)
        tm = heat_step(tm, MOTOR_HEAT_SHARE * abs(p) * (1 - veh.eff), veh.motor_cool, MOTOR_THERMAL_MASS, air, dt)
        ch["batt_temp"].append(tb); ch["motor_temp"].append(tm)
        tf_sum += tf * dt; tr_sum += tr * dt
        sl_f += st["slide_f"] * dt; sl_r += st["slide_r"] * dt
        t += dt
    return dict(time=t, q_ah=q_ah, e_out_kwh=e_out / 3.6e6, e_regen_kwh=e_regen / 3.6e6, ch=ch,
                temp_end=(tf, tr), pt_end=(tb, tm), temp_mean=(tf_sum / t, tr_sum / t), slide_j=(sl_f, sl_r))


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
            slip_f=abs(ch["slip_f"][apex]), slip_r=abs(ch["slip_r"][apex]), steer=abs(ch["steer"][apex]), cam_fo=ch["cam_fo"][apex], cam_ro=ch["cam_ro"][apex],
        ))
    return corners


def _time_at(ch, ds, s):
    i = min(len(ch["t"]) - 1, int(s / ds))
    return ch["t"][i] + (s - i * ds) / max(ch["v"][i], 0.15)


def _round_ch(ch):
    dec = dict(t=3, v=2, ax=3, ay=3, p=2, torque=1, uf=3, ur=3, roll=2, slip_f=2, slip_r=2, steer=2, slip_ratio=2, temp_f=1, temp_r=1, cam_fo=2, cam_ro=2, batt_temp=2, motor_temp=2, tv=0)
    return {k: [round(x, dec.get(k, 0)) for x in a] for k, a in ch.items()}


FEEDBACK_ITERS = 4     # times the lap is re-solved with updated transient tyre loads
FEEDBACK_RELAX = 0.5   # how much of each update is taken (damps the back-and-forth)
SMOOTH_M = 1.0         # m, light smoothing of the load differences along the track
WHEELS = ("fl", "fr", "rl", "rr")


def _settled_loads(veh, ch, j):
    return veh.wheel_loads(ch["v"][j], ch["ax"][j] * G, ch["ay"][j] * G)


def _dynamic_offsets(veh, track, standing):
    """Couple the lap solver to the spring and damper model.

    1. Solve the lap with settled tyre loads.
    2. Replay it through the spring and damper model to get the real,
       time-varying tyre loads.
    3. Note, at every point on the track, how far each tyre's load is from its
       settled value, and solve the lap again with those differences applied.
    Repeat a few times. Returns the load differences, the corner speed limits
    without them, and the lap time after each round (first entry = settled).
    """
    ks = track["k"]
    base = _corner_limits(veh, ks)
    offs = None
    times = []
    for it in range(FEEDBACK_ITERS + 1):
        vlim = base if offs is None else _corner_limits(veh, ks, offs, base)
        res = _integrate(veh, track, _solve_lap(veh, track, vlim, standing, offs), offs)
        times.append(res["time"])
        if it == FEEDBACK_ITERS:
            final = (vlim, res)
            break
        ch = res["ch"]
        rch, _ = ride_replay(veh, ch["t"], ch["v"], ch["ax"], ch["ay"])
        last = len(ch["v"]) - 1
        new = []
        for i in range(len(ks)):
            j = min(i, last)
            st = _settled_loads(veh, ch, j)
            d = tuple(rch["fzd_" + w][j] - st[q] for q, w in enumerate(WHEELS))
            if offs is not None:
                d = tuple(o + FEEDBACK_RELAX * (x - o) for o, x in zip(offs[i], d))
            new.append(d)
        # Light smoothing along the track (about 3 m). Braking points shift a little
        # between rounds; without this the sharp load changes there never settle.
        w = max(1, int(round(SMOOTH_M / 2 / track["ds"])))
        n = len(new)
        if track["closed"]:
            sm = [tuple(sum(new[(i + q) % n][c] for q in range(-w, w + 1)) / (2 * w + 1) for c in range(4)) for i in range(n)]
        else:
            sm = [tuple(sum(new[min(n - 1, max(0, i + q))][c] for q in range(-w, w + 1)) / (2 * w + 1) for c in range(4)) for i in range(n)]
        offs = sm
    return offs, final, times


def simulate(car, cond, track):
    """Run one event. Returns a plain dict ready to be turned into JSON."""
    veh = Vehicle(car, cond)
    track = dict(track)
    track["k"] = list(track["k"])
    # Each solve starts its searches from the answers of the solve before (same
    # place on the track), which is most of the speed of a multi-lap run.
    veh.hint_a = [None] * len(track["k"]); veh.hint_b = [None] * len(track["k"])
    ds = track["ds"]
    laps = track.get("laps", 1)
    standing = track.get("standing_start", False)
    soc0 = veh.soc
    wear0 = veh.wear
    temp0 = veh.temp_f
    pt0 = (veh.batt_temp, veh.motor_temp)
    if laps == 1:
        # Tyre grip for a single lap uses the average tyre temperature over that
        # lap. Two quick solves find it: solve, read the temperatures, solve again.
        for _ in range(2):
            pre = _integrate(veh, track, _solve_lap(veh, track, _corner_limits(veh, track["k"]), standing))
            veh.set_tyres(wear0, wear0, *pre["temp_mean"])
    lap_km = len(track["k"]) * ds / 1000.0
    lap_rows = []
    total = 0.0; e_out = 0.0; e_regen = 0.0
    shown = None
    finished = True
    vlim = None; vlim_grip = None
    # Tyre load differences caused by body movement on the springs and dampers.
    # For multi-lap events they are worked out once, on a flying lap.
    offs, fb_final, fb_times = _dynamic_offsets(veh, track, standing and laps == 1)
    for lap in range(1, laps + 1):
        if veh.soc <= SOC_MIN:
            finished = False
            break
        # Corner speed limits only change with tyre wear; refresh every 2 % of wear.
        if laps == 1:
            vlim, res = fb_final                 # already solved in the feedback loop
        else:
            if vlim is None:
                vlim = fb_final[0]; vlim_grip = (veh.grip_f, veh.grip_r)
            elif abs(veh.grip_f / vlim_grip[0] - 1) > 0.005 or abs(veh.grip_r / vlim_grip[1] - 1) > 0.005:
                # corner speed limits only change with tyre grip (wear and temperature)
                vlim = _corner_limits(veh, track["k"], offs, vlim); vlim_grip = (veh.grip_f, veh.grip_r)
            res = _integrate(veh, track, _solve_lap(veh, track, vlim, standing and lap == 1, offs), offs)
        p_avail = veh.p_elec / 1000.0
        soc_end = veh.soc - res["q_ah"] / veh.cap_ah
        total += res["time"]; e_out += res["e_out_kwh"]; e_regen += res["e_regen_kwh"]
        lap_rows.append(dict(lap=lap, time=round(res["time"], 3), soc_end=round(soc_end * 100, 2),
                             e_kwh=round(res["e_out_kwh"] - res["e_regen_kwh"], 4),
                             p_avail=round(p_avail, 1), wear=round(veh.wear, 2),
                             wear_f=round(veh.wear_f, 2), wear_r=round(veh.wear_r, 2),
                             temp_f=round(res["temp_end"][0], 1), temp_r=round(res["temp_end"][1], 1),
                             batt_temp=round(res["pt_end"][0], 1), motor_temp=round(res["pt_end"][1], 1),
                             batt_cut=round(100 * veh.batt_cut), motor_cut=round(100 * veh.motor_cut),
                             grip_f=round(100 * veh.grip_f / veh.track_grip, 1), grip_r=round(100 * veh.grip_r / veh.track_grip, 1)))
        # Show the first flying lap for multi-lap events, otherwise the only lap.
        if shown is None and (laps == 1 or lap == 2):
            shown = res; shown_lap = lap
        if laps > 1:
            veh.batt_temp, veh.motor_temp = res["pt_end"]
        veh.set_state(soc_end)
        if laps > 1:
            # carry tyre temperature and wear into the next lap
            wf = veh.wear_f + wear_added(veh.wear_rate, res["slide_j"][0], res["temp_mean"][0], veh.t_opt) * 1.0
            wr = veh.wear_r + wear_added(veh.wear_rate, res["slide_j"][1], res["temp_mean"][1], veh.t_opt) * 1.0
            veh.temp_f, veh.temp_r = res["temp_end"]
            veh.set_tyres(wf, wr, *res["temp_end"])
        last = res
    if shown is None:
        shown = res; shown_lap = 1
    ch = shown["ch"]
    # Replay the lap through the spring and damper model (see ride.py).
    tyres = dict(temp_start=temp0, temp_f_end=round(last["temp_end"][0], 1), temp_r_end=round(last["temp_end"][1], 1),
                 temp_f_mean=round(shown["temp_mean"][0], 1), temp_r_mean=round(shown["temp_mean"][1], 1),
                 temp_opt=veh.t_opt, wear_f=round(veh.wear_f, 2), wear_r=round(veh.wear_r, 2),
                 slide_kj_f=round(shown["slide_j"][0] / 1000.0, 1), slide_kj_r=round(shown["slide_j"][1] / 1000.0, 1),
                 grip_f=round(100 * temp_grip(shown["temp_mean"][0], veh.t_opt), 1),
                 grip_r=round(100 * temp_grip(shown["temp_mean"][1], veh.t_opt), 1))
    heat = dict(batt_start=pt0[0], motor_start=pt0[1], batt_end=round(last["pt_end"][0], 1), motor_end=round(last["pt_end"][1], 1),
                batt_peak=round(max([pt0[0]] + [r["batt_temp"] for r in lap_rows] + ch["batt_temp"]), 1),
                motor_peak=round(max([pt0[1]] + [r["motor_temp"] for r in lap_rows] + ch["motor_temp"]), 1),
                batt_cut_min=min([100] + [r["batt_cut"] for r in lap_rows]),
                motor_cut_min=min([100] + [r["motor_cut"] for r in lap_rows]))
    veh.batt_temp, veh.motor_temp = pt0
    veh.set_state(soc0)
    veh.temp_f = veh.temp_r = temp0
    ride_ch, ride = ride_replay(veh, ch["t"], ch["v"], ch["ax"], ch["ay"])
    for q, w in enumerate(WHEELS):          # time-based load minus settled load, for display
        ride_ch["dfz_" + w] = [round(ride_ch["fzd_" + w][j] - _settled_loads(veh, ch, j)[q]) for j in range(len(ch["v"]))]
    bm = bump_metrics(veh, sum(ch["v"]) / len(ch["v"]))
    ride.update({k: (round(x, 3) if isinstance(x, float) else x) for k, x in bm.items()})
    ride["time_settled"] = round(fb_times[0], 3)
    ride["time_dynamic"] = round(fb_times[-1], 3)
    ride["time_cost"] = round(fb_times[-1] - fb_times[0], 3)
    ride["feedback_times"] = [round(x, 3) for x in fb_times]
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
        ch=_round_ch(ch), ride=ride, tyres=tyres, heat=heat,
    )
    out["ch"].update(ride_ch)
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
