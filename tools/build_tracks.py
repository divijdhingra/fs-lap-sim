"""Builds data/tracks.json, the track file the simulator reads.

Run:  python3 tools/build_tracks.py [path/to/Onboarding_GPS.mat]

Every track ends up in the same simple form: a list of points spaced evenly
along the driving line, each with a position (x, y in metres) and a curvature
(1/radius in 1/m, positive = left turn).

Sources
- Acceleration and skidpad: drawn from the Formula Student rulebook dimensions.
- Silverstone GP and Hockenheim GP: racing lines from the TUM racetrack
  database (https://github.com/TUMFTM/racetrack-database, LGPL-3.0).
- FS UK endurance layout: the average driven line from a logged GPS stint.
  The raw log is NOT stored in this repository; if the .mat file is not
  supplied, the existing layout in data/tracks.json is kept.
- Real competition cone layouts: cone positions in data/raw/cones/ (see the
  README there for where each came from). The middle of the lane is found
  from the two rows of cones, then fslap/layout.py works out the driven line.
"""
import json, math, os, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap.layout import build_track
OUT = os.path.join(ROOT, "data", "tracks.json")


def smooth_closed(a, sigma_pts):
    """Gaussian smoothing on a closed loop (wraps round the ends)."""
    if sigma_pts <= 0:
        return a
    n = int(4 * sigma_pts) + 1
    w = np.exp(-0.5 * (np.arange(-n, n + 1) / sigma_pts) ** 2)
    w /= w.sum()
    ext = np.concatenate([a[-n:], a, a[:n]])
    return np.convolve(ext, w, mode="same")[n:-n]


def smooth_open(a, sigma_pts):
    if sigma_pts <= 0:
        return a
    n = int(4 * sigma_pts) + 1
    w = np.exp(-0.5 * (np.arange(-n, n + 1) / sigma_pts) ** 2)
    w /= w.sum()
    ext = np.concatenate([np.full(n, a[0]), a, np.full(n, a[-1])])
    return np.convolve(ext, w, mode="same")[n:-n]


def resample_closed(x, y, ds):
    """Evenly spaced points round a closed loop. Returns x, y, actual ds."""
    xc, yc = np.append(x, x[0]), np.append(y, y[0])
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(xc), np.diff(yc)))])
    n = int(round(s[-1] / ds))
    si = np.arange(n) * (s[-1] / n)
    return np.interp(si, s, xc), np.interp(si, s, yc), s[-1] / n


def curvature_closed(x, y, ds):
    dx = (np.roll(x, -1) - np.roll(x, 1)) / (2 * ds)
    dy = (np.roll(y, -1) - np.roll(y, 1)) / (2 * ds)
    ddx = (np.roll(x, -1) - 2 * x + np.roll(x, 1)) / ds ** 2
    ddy = (np.roll(y, -1) - 2 * y + np.roll(y, 1)) / ds ** 2
    return (dx * ddy - dy * ddx) / (dx * dx + dy * dy) ** 1.5


def closed_track(x, y, ds, sigma_m):
    """Resample, smooth the line, resample again, then take curvature."""
    x, y, d = resample_closed(np.asarray(x), np.asarray(y), ds)
    x, y = smooth_closed(x, sigma_m / d), smooth_closed(y, sigma_m / d)
    x, y, d = resample_closed(x, y, ds)
    k = smooth_closed(curvature_closed(x, y, d), 1.0)
    return x, y, k, d


def pack(tid, name, kind, x, y, k, ds, closed, **meta):
    t = dict(id=tid, name=name, kind=kind, closed=closed, ds=round(float(ds), 5),
             length=round(float(ds) * len(x), 2),
             x=[round(float(v), 2) for v in x], y=[round(float(v), 2) for v in y],
             k=[round(float(v), 5) for v in k])
    t.update(meta)
    return t


def from_segments(segs, ds, x0=0.0, y0=0.0, h0=math.pi / 2):
    """Draw a path from ('straight', length) and ('arc', radius, angle_rad)
    pieces. Positive angle = left turn."""
    xs, ys, ks = [x0], [y0], [0.0]
    x, y, h = x0, y0, h0
    for seg in segs:
        if seg[0] == "straight":
            L, kap = seg[1], 0.0
        else:
            L, kap = abs(seg[1] * seg[2]), math.copysign(1.0 / seg[1], seg[2])
        n = max(1, int(round(L / ds)))
        step = L / n
        for _ in range(n):
            h_mid = h + kap * step / 2
            x += step * math.cos(h_mid); y += step * math.sin(h_mid); h += kap * step
            xs.append(x); ys.append(y); ks.append(kap)
    return np.array(xs), np.array(ys), np.array(ks)


def build_accel():
    ds = 0.25
    n = int(75 / ds) + 1
    return pack("accel", "Acceleration (75 m)", "accel", np.zeros(n), np.arange(n) * ds,
                np.zeros(n), ds, False, laps=1, standing_start=True, corner_radius=0,
                source="Formula Student rules: 75 m straight from a standing start.")


def build_skidpad():
    # FS rules: two circles, centres 18.25 m apart, inner diameter 15.25 m,
    # 3 m wide lane -> the middle of the lane is a circle of radius 9.125 m.
    R, ds, lead = 9.125, 0.25, 15.0
    two = 4 * math.pi
    segs = [("straight", lead), ("arc", R, -two), ("arc", R, two), ("straight", lead)]
    x, y, k = from_segments(segs, ds, 0.0, -lead)
    # exact step used by from_segments differs slightly per piece; use mean
    d = float(np.mean(np.hypot(np.diff(x), np.diff(y))))
    circ = 2 * math.pi * R
    k = smooth_open(k, 2.0)
    return pack("skidpad", "Skidpad (figure of eight)", "skidpad", x, y, k, d, False,
                laps=1, standing_start=False, entry_speed=10.0, corner_radius=0,
                timed=[[lead + circ, lead + 2 * circ], [lead + 3 * circ, lead + 4 * circ]],
                source="Formula Student rules: two 15.25 m inner-diameter circles, 3 m lane. "
                       "Driven line = lane centre (radius 9.125 m). Timed laps are the second "
                       "lap of each circle, averaged.")


def build_gp(tid, name, fname):
    raw = np.loadtxt(os.path.join(ROOT, "data", "raw", fname), delimiter=",", comments="#")
    x, y, k, d = closed_track(raw[:, 0], raw[:, 1], 2.0, 6.0)
    return pack(tid, name, "circuit", x, y, k, d, True, laps=1, standing_start=False,
                corner_radius=220,
                source="Racing line from the TUM racetrack database (LGPL-3.0), "
                       "smoothed and resampled every 2 m.")


def build_fsuk(mat):
    import scipy.io as sio
    g = sio.loadmat(mat)
    t, lat, lon = (g[n].squeeze() for n in ("GPStime", "GPSlat", "GPSlon"))
    lat0, lon0, Re = 52.07806, -1.011115, 6371000.0
    x = np.radians(lon - lon0) * Re * math.cos(math.radians(lat0))
    y = np.radians(lat - lat0) * Re
    ls, le = np.array([-1.0111, 52.07812]), np.array([-1.01113, 52.07800])
    ld = le - ls
    side = ld[0] * (lat - ls[1]) - ld[1] * (lon - ls[0])
    k = np.where(side[:-1] * side[1:] < 0)[0]
    k = np.array([i for i in k if side[i] < 0 and abs(x[i]) < 15 and abs(y[i]) < 15])
    # Average all laps on a common "fraction of lap distance" axis to beat down GPS noise.
    N = 1600
    acc_x, acc_y = np.zeros(N), np.zeros(N)
    for a, b in zip(k[:-1], k[1:]):
        xs, ys = x[a:b + 1], y[a:b + 1]
        s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
        f = np.arange(N) / N * s[-1]
        acc_x += np.interp(f, s, xs); acc_y += np.interp(f, s, ys)
    n = len(k) - 1
    X, Y, K, d = closed_track(acc_x / n, acc_y / n, 0.5, 2.0)
    return X, Y, K, d


def order_loop(P):
    """Put one row of cones in order round the lap: always step to the nearest
    cone not yet visited."""
    P = np.asarray(P, float); left = list(range(1, len(P))); o = [0]
    while left:
        d = np.hypot(*(P[left] - P[o[-1]]).T)
        o.append(left.pop(int(np.argmin(d))))
    return P[o]


def dense_loop(P, ds=0.25):
    x, y, _ = resample_closed(P[:, 0], P[:, 1], ds)
    return np.column_stack([x, y])


def lane_middle(left, right, start, heading_deg):
    """Middle of the lane between two rows of cones, as points every 1 m in
    driving order from the start line, with half the lane width at each."""
    L, R = dense_loop(order_loop(left)), dense_loop(order_loop(right))
    # first guess: halfway from each left-row point to the nearest right-row point
    j = [int(np.argmin(np.hypot(*(R - p).T))) for p in L]
    C = (L + R[j]) / 2
    x, y, d = resample_closed(C[:, 0], C[:, 1], 1.0)
    # which side of the point order the left row is on (the cone order may run backwards)
    p0, p1 = np.array([x[0], y[0]]), np.array([x[1], y[1]])
    l0 = L[int(np.argmin(np.hypot(*(L - p0).T)))]
    side = 1.0 if (p1[0] - p0[0]) * (l0[1] - p0[1]) - (p1[1] - p0[1]) * (l0[0] - p0[0]) > 0 else -1.0
    for _ in range(6):                      # slide each point until it is the same distance from both rows
        x, y = smooth_closed(x, 1.0), smooth_closed(y, 1.0)
        tx, ty = np.roll(x, -1) - np.roll(x, 1), np.roll(y, -1) - np.roll(y, 1)
        n = np.hypot(tx, ty); nx, ny = -ty / n, tx / n
        P = np.column_stack([x, y])
        dl = np.array([np.hypot(*(L - p).T).min() for p in P]); dr = np.array([np.hypot(*(R - p).T).min() for p in P])
        x, y = x + side * nx * (dl - dr) / 2, y + side * ny * (dl - dr) / 2
        x, y, d = resample_closed(x, y, 1.0)
    P = np.column_stack([x, y])
    dl = np.array([np.hypot(*(L - p).T).min() for p in P]); dr = np.array([np.hypot(*(R - p).T).min() for p in P])
    print("   lane middle off-centre by at most %.2f m, lane width %.1f to %.1f m" % (np.abs(dl - dr).max() / 2, (dl + dr).min(), (dl + dr).max()))
    half = (dl + dr) / 2
    # be cautious about the room: the narrowest the lane gets within 3 m either way
    half = np.min([np.roll(half, q) for q in range(-3, 4)], axis=0)
    # start at the start line and run in the driving direction
    i0 = int(np.argmin(np.hypot(x - start[0], y - start[1])))
    h = math.radians(heading_deg)
    fwd = (x[(i0 + 3) % len(x)] - x[i0]) * math.cos(h) + (y[(i0 + 3) % len(x)] - y[i0]) * math.sin(h)
    idx = (i0 + np.arange(len(x))) % len(x) if fwd > 0 else (i0 - np.arange(len(x))) % len(x)
    return x[idx], y[idx], half[idx]


def build_cones(fname):
    c = json.load(open(os.path.join(ROOT, "data", "raw", "cones", fname)))
    x, y, half = lane_middle(c["left"], c["right"], c["start"], c["heading_deg"])
    cones = dict(left=c["left"], right=c["right"], orange=c["orange"])
    t = build_track([float(v) for v in x], [float(v) for v in y], [float(v) for v in half], c["id"], c["name"],
                    source=c["origin"] + " " + c["licence"] + ". Driven line: the least-bent line that keeps the car "
                    "0.85 m from its centre to the cones.", cones=cones)
    t["group"] = "cones"
    t["lane_width"] = round(float(2 * np.median(half)), 2)
    return t


def main():
    old = {}
    if os.path.exists(OUT):
        old = {t["id"]: t for t in json.load(open(OUT))["tracks"]}
    tracks = [build_accel(), build_skidpad()]
    if len(sys.argv) > 1:
        X, Y, K, d = build_fsuk(sys.argv[1])
        src = ("Average driven line from a logged FS UK 2025 endurance stint at Silverstone "
               "(GPS, 10 Hz, 13 laps averaged, smoothed, resampled every 0.5 m).")
        tracks.append(pack("fsuk_autocross", "FS UK Sprint / Autocross (1 lap)", "autocross", X, Y, K, d,
                           True, laps=1, standing_start=True, corner_radius=45, source=src +
                           " Assumption: the sprint uses the same layout as endurance."))
        tracks.append(pack("fsuk_endurance", "FS UK Endurance (27 laps, 22 km)", "endurance", X, Y, K, d,
                           True, laps=27, standing_start=True, corner_radius=45, source=src))
    else:
        for tid in ("fsuk_autocross", "fsuk_endurance"):
            if tid in old:
                tracks.append(old[tid])
    for f in ("fsg18.json", "fsi18.json", "fsg19.json", "fss19.json", "fse22.json", "fsg23.json"):
        tracks.append(build_cones(f))
    tracks.append(build_gp("silverstone_gp", "Silverstone GP", "silverstone_gp_raceline.csv"))
    tracks.append(build_gp("hockenheim_gp", "Hockenheim GP", "hockenheim_gp_raceline.csv"))
    json.dump({"tracks": tracks}, open(OUT, "w"), separators=(",", ":"))
    for t in tracks:
        kk = np.abs(np.array(t["k"]))
        print(f'{t["id"]:16s} {t["length"]:8.1f} m  ds={t["ds"]}  n={len(t["x"])}  min radius={1 / kk.max() if kk.max() > 0 else float("inf"):.1f} m')


if __name__ == "__main__":
    main()
