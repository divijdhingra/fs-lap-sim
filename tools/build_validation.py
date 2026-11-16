"""Builds data/validation.json: the logged laps the simulator is checked against.

Run:  python3 tools/build_validation.py "path/to/folder with the Onboarding_*.mat files"

Only summary figures and the fastest lap's speed and power traces (resampled
onto the track's distance steps) are stored. The raw logs stay out of the
repository.
"""
import json, math, os, sys
import numpy as np
import scipy.io as sio
from scipy.signal import savgol_filter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main(folder):
    g = sio.loadmat(os.path.join(folder, "Onboarding_GPS.mat"))
    t, lat, lon = (g[n].squeeze() for n in ("GPStime", "GPSlat", "GPSlon"))
    p = sio.loadmat(os.path.join(folder, "Onboarding_Power.mat"))
    pw, pt = p["Power"].squeeze(), p["PowerTime"].squeeze()
    lat0, lon0, Re = 52.07806, -1.011115, 6371000.0
    x = np.radians(lon - lon0) * Re * math.cos(math.radians(lat0))
    y = np.radians(lat - lat0) * Re
    ls, le = np.array([-1.0111, 52.07812]), np.array([-1.01113, 52.07800])
    ld = le - ls
    side = ld[0] * (lat - ls[1]) - ld[1] * (lon - ls[0])
    k = np.where(side[:-1] * side[1:] < 0)[0]
    k = np.array([i for i in k if side[i] < 0 and abs(x[i]) < 15 and abs(y[i]) < 15])
    frac = side[k] / (side[k] - side[k + 1])
    ct = t[k] + frac * (t[k + 1] - t[k])
    lap_times = np.diff(ct)
    dt = 0.1
    vx = savgol_filter(x, 11, 3, deriv=1, delta=dt); vy = savgol_filter(y, 11, 3, deriv=1, delta=dt)
    ax = savgol_filter(x, 15, 3, deriv=2, delta=dt); ay = savgol_filter(y, 15, 3, deriv=2, delta=dt)
    v = np.hypot(vx, vy)
    along = (ax * vx + ay * vy) / np.maximum(v, 0.5) / 9.81
    latg = (vx * ay - vy * ax) / np.maximum(v, 0.5) / 9.81
    energy = []
    for a, b in zip(ct[:-1], ct[1:]):
        m = (pt >= a) & (pt < b)
        energy.append(float(np.trapezoid(pw[m], pt[m]) / 3600))
    track = [tr for tr in json.load(open(os.path.join(ROOT, "data", "tracks.json")))["tracks"] if tr["id"] == "fsuk_endurance"][0]
    n = len(track["x"])
    best = int(np.argmin(lap_times))
    a, b = k[best], k[best + 1]
    s = np.concatenate([[0], np.cumsum(v[a:b] * dt)])[: b - a + 1]
    f = s / s[-1] * n                                    # logged distance mapped onto track steps
    idx = np.arange(n)
    v_tr = np.interp(idx, f, v[a:b + 1])
    p_tr = np.interp(idx, f, np.interp(t[a:b + 1], pt, pw))
    inlap = (t >= ct[0]) & (t <= ct[-1]) & (v > 3)
    out = dict(
        source="FS UK 2025 endurance stint at Silverstone, 10 Hz GPS and power logger, 13 timed laps.",
        lap_times=[round(float(q), 2) for q in lap_times],
        lap_energy_kwh=[round(q, 4) for q in energy],
        best_lap=best + 1, best_time=round(float(lap_times[best]), 2),
        v=[round(float(q), 2) for q in v_tr], p=[round(float(q), 1) for q in p_tr],
        peak_lat_g=round(float(np.percentile(np.abs(latg[inlap]), 99)), 2),
        peak_brake_g=round(float(np.percentile(-along[inlap], 99)), 2),
        peak_accel_g=round(float(np.percentile(along[inlap], 99)), 2),
        peak_power_kw=round(float(pw.max()), 1),
    )
    json.dump(out, open(os.path.join(ROOT, "data", "validation.json"), "w"), separators=(",", ":"))
    print({k2: v2 for k2, v2 in out.items() if not isinstance(v2, list)}, "best lap energy", out["lap_energy_kwh"][best])


if __name__ == "__main__":
    main(sys.argv[1])
