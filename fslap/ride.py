"""Time-based spring and damper model ("ride replay").

The lap solver assumes the car is always settled. This module takes the
finished lap (acceleration, braking and cornering against time) and replays it
through a car body sitting on springs and dampers, stepping forward a few
thousandths of a second at a time. It shows what the steady model cannot:
how fast the body rolls, dives and squats, how hard the dampers are worked,
and how tyre loads overshoot or lag when the driver brakes or turns in.

Three body motions are simulated:
  roll   leaning sideways in corners
  pitch  nose down under braking (dive), nose up under power (squat)
  heave  the whole body moving down on its springs as downforce builds

Assumptions
- The replay does not feed back into lap time. It is an analysis of the lap,
  not a re-run of it.
- Roll centres and pitch centre at ground level (same as the lap solver, so
  the settled values agree exactly with it).
- Dampers are linear: the same rate in bump and rebound, at all speeds.
- Tyres are rigid vertically and the road is perfectly smooth, so only
  driver-induced movement is seen, not bumps or kerbs.
- Driver inputs build up with a short lag (INPUT_LAG seconds) rather than
  instantly, standing in for tyre response time.
- Roll and pitch inertia come from assumed radii of gyration (KX, KY).
"""
import math
from .bumps import tyre_load_rms, hop_frequency

G = 9.81
KX, KY = 0.30, 0.60        # m, radii of gyration of the sprung mass in roll and pitch
INPUT_LAG = 0.08           # s
BINS = [25, 50, 100, 200]  # damper speed bin edges, mm/s


def ride_metrics(veh):
    """Design numbers a suspension engineer checks first."""
    out = {}
    for tag, k, c, share in (("f", veh.kw_f, veh.c_f, veh.wf), ("r", veh.kw_r, veh.c_r, 1 - veh.wf)):
        ms = max(5.0, veh.m * share / 2 - veh.m_unsprung)      # sprung mass on one corner
        out["freq_" + tag] = math.sqrt(k / ms) / (2 * math.pi)
        out["zeta_" + tag] = c / (2 * math.sqrt(k * ms))
    ixx = veh.m_sprung * KX ** 2
    out["freq_roll"] = math.sqrt(veh.kroll_f + veh.kroll_r) / math.sqrt(ixx) / (2 * math.pi)
    out["zeta_roll"] = (veh.croll_f + veh.croll_r) / (2 * math.sqrt((veh.kroll_f + veh.kroll_r) * ixx))
    out["roll_gradient"] = math.degrees(veh.m * G * veh.h / (veh.kroll_f + veh.kroll_r))   # deg per g
    return out


SWEEP = [0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.65, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0]


def bump_metrics(veh, v_mean):
    """Tyre load flutter over bumps at the lap's average speed, plus how it
    would change with softer or stiffer dampers (the classic damper trade-off)."""
    mu, kt, gd0 = veh.m_unsprung, veh.kt, veh.gd0
    static_f, static_r = veh.m * G * veh.wf / 2, veh.m * G * (1 - veh.wf) / 2
    rv = math.sqrt(max(v_mean, 0.1))
    out = dict(
        hop_f=hop_frequency(mu, veh.kw_f, kt), hop_r=hop_frequency(mu, veh.kw_r, kt),
        flutter_f=100 * veh.sig_f * rv / static_f, flutter_r=100 * veh.sig_r * rv / static_r,
        v_mean=v_mean,
    )
    zeta, var_f, var_r = [], [], []
    for mult in SWEEP:
        zeta.append(round(mult * veh.c_f / (2 * math.sqrt(veh.kw_f * veh.ms_f)), 3))
        var_f.append(round(100 * tyre_load_rms(veh.ms_f, mu, veh.kw_f, mult * veh.c_f, kt, v_mean, gd0) / static_f, 2))
        var_r.append(round(100 * tyre_load_rms(veh.ms_r, mu, veh.kw_r, mult * veh.c_r, kt, v_mean, gd0) / static_r, 2))
    out["sweep"] = dict(mult=SWEEP, zeta=zeta, var_f=var_f, var_r=var_r)
    return out


def ride_replay(veh, t, v, ax_g, ay_g, lag=INPUT_LAG):
    """Replay a lap. t, v, ax_g, ay_g are per track step (s, m/s, g, g).
    Returns per-step channels and a summary."""
    m, h, L = veh.m, veh.h, veh.L
    ms = veh.m_sprung
    a, b = L * (1 - veh.wf), L * veh.wf                 # CoG to front / rear axle
    kf, kr, cf, cr = veh.kw_f, veh.kw_r, veh.c_f, veh.c_r
    Kf, Kr, Cf, Cr = veh.kroll_f, veh.kroll_r, veh.croll_f, veh.croll_r
    tf, tr = veh.tf, veh.tr
    ixx, iyy = ms * KX ** 2, ms * KY ** 2
    wf_static, wr_static = m * G * veh.wf, m * G * (1 - veh.wf)
    # keep the explicit time step small compared with the fastest damping rate
    rate = max((Cf + Cr) / ixx, 2 * (cf + cr) / ms, 2 * (cf * a * a + cr * b * b) / iyy, 1.0)
    h_max = min(0.004, 0.4 / rate)

    # start settled at the first sample
    ax, ay = ax_g[0] * G, ay_g[0] * G
    dn = veh.q_cl * v[0] ** 2
    ff, fr = dn * veh.ab - m * ax * h / L, dn * (1 - veh.ab) + m * ax * h / L
    df_, dr_ = ff / (2 * kf), fr / (2 * kr)
    th, z = (df_ - dr_) / L, -(df_ * b + dr_ * a) / L
    phi = m * ay * h / (Kf + Kr)
    thd = zd = phid = 0.0

    n = len(t)
    ch = dict(roll_dyn=[], pitch=[], sus_f=[], sus_r=[],
              fzd_fl=[], fzd_fr=[], fzd_rl=[], fzd_rr=[], dv_fl=[], dv_fr=[], dv_rl=[], dv_rr=[])
    hist_f = [0.0] * (len(BINS) + 1); hist_r = [0.0] * (len(BINS) + 1)
    total_t = 0.0
    for i in range(n):
        dt = (t[i + 1] - t[i]) if i + 1 < n else (t[i] - t[i - 1] if i else 0.01)
        dt = min(max(dt, 1e-4), 2.0)
        steps = max(1, int(math.ceil(dt / h_max)))
        hs = dt / steps
        ax_t, ay_t = ax_g[i] * G, ay_g[i] * G
        dn = veh.q_cl * v[i] ** 2
        dnf, dnr = dn * veh.ab, dn * (1 - veh.ab)
        alpha = 1.0 if lag <= 0 else 1 - math.exp(-hs / lag)
        for _ in range(steps):
            ax += (ax_t - ax) * alpha
            ay += (ay_t - ay) * alpha
            d_f, d_r = -z + a * th, -z - b * th                  # suspension compression, m
            v_f, v_r = -zd + a * thd, -zd - b * thd              # compression speed, m/s
            ff = 2 * (kf * d_f + cf * v_f)                       # axle force change from springs + dampers
            fr = 2 * (kr * d_r + cr * v_r)
            zdd = (ff + fr - dn) / ms
            thdd = (-m * ax * h - a * ff + b * fr + a * dnf - b * dnr) / iyy
            phidd = (m * ay * h - (Kf + Kr) * phi - (Cf + Cr) * phid) / ixx
            zd += zdd * hs; thd += thdd * hs; phid += phidd * hs
            z += zd * hs; th += thd * hs; phi += phid * hs
        d_f, d_r = -z + a * th, -z - b * th
        v_f, v_r = -zd + a * thd, -zd - b * thd
        ff = 2 * (kf * d_f + cf * v_f); fr = 2 * (kr * d_r + cr * v_r)
        lt_f = (Kf * phi + Cf * phid) / tf                        # lateral load transfer, front axle
        lt_r = (Kr * phi + Cr * phid) / tr
        axf, axr = max(0.0, wf_static + ff), max(0.0, wr_static + fr)
        lt_f = max(-axf / 2, min(axf / 2, lt_f)); lt_r = max(-axr / 2, min(axr / 2, lt_r))
        dv = (v_f - phid * tf / 2, v_f + phid * tf / 2, v_r - phid * tr / 2, v_r + phid * tr / 2)
        ch["roll_dyn"].append(round(math.degrees(phi), 3))
        ch["pitch"].append(round(math.degrees(th), 3))
        ch["sus_f"].append(round(d_f * 1000, 2)); ch["sus_r"].append(round(d_r * 1000, 2))
        ch["fzd_fl"].append(round(axf / 2 - lt_f)); ch["fzd_fr"].append(round(axf / 2 + lt_f))
        ch["fzd_rl"].append(round(axr / 2 - lt_r)); ch["fzd_rr"].append(round(axr / 2 + lt_r))
        for key, val in zip(("dv_fl", "dv_fr", "dv_rl", "dv_rr"), dv):
            ch[key].append(round(val * 1000, 1))
        for hist, vals in ((hist_f, dv[:2]), (hist_r, dv[2:])):
            for val in vals:
                s = abs(val) * 1000
                j = 0
                while j < len(BINS) and s >= BINS[j]:
                    j += 1
                hist[j] += dt
        total_t += dt
    summ = ride_metrics(veh)
    summ.update(
        roll_max=max(abs(x) for x in ch["roll_dyn"]),
        dive_max=max(0.0, max(ch["pitch"])), squat_max=max(0.0, -min(ch["pitch"])),
        sus_f_max=max(ch["sus_f"]), sus_r_max=max(ch["sus_r"]),
        dv_f_max=max(max(abs(x) for x in ch["dv_fl"]), max(abs(x) for x in ch["dv_fr"])),
        dv_r_max=max(max(abs(x) for x in ch["dv_rl"]), max(abs(x) for x in ch["dv_rr"])),
        fz_min=min(min(ch[k]) for k in ("fzd_fl", "fzd_fr", "fzd_rl", "fzd_rr")),
        bins=BINS,
        hist_f=[round(100 * x / (2 * total_t), 2) for x in hist_f],
        hist_r=[round(100 * x / (2 * total_t), 2) for x in hist_r],
    )
    return ch, {k: (round(x, 3) if isinstance(x, float) else x) for k, x in summ.items()}
