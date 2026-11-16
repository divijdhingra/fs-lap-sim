"""Turns a lane marked by cones into the line the car drives.

A cone layout gives a lane, not a line. A real driver does not follow the
middle of the lane: they use its width to open the corners out. This file
works out that line.

Method (plain English)
1. Take the middle of the lane as evenly spaced points, each with the room
   the car has either side of it (half the lane width, less half the car and
   a safety margin to the cones).
2. Let every point slide sideways across the lane, but never past that room.
3. Repeatedly nudge stretches of the line a little to the left and a little
   to the right, and keep whichever makes the line less bent without making
   it much longer, in smaller and smaller steps until nothing improves.
   Bends cost lap time, so the line straightens itself as far as the cones
   allow: wide on the way in, close to the inside cones mid-corner, wide on
   the way out. Distance counts a little too (LENGTH_WEIGHT), which stops the
   line wandering on straights.
4. Re-space the points evenly and measure how tightly the line bends at each
   point (curvature = 1 / corner radius), which is what the lap solver needs.

Assumptions
- The line is the same every lap. It is found from the shape of the lane
  alone, not from the car, so it is a good line rather than the single
  fastest line for one particular car. Changing how much distance counts
  moved the lap time by under 2% on the layouts tried.
- The car needs CAR_HALF_WIDTH + CONE_MARGIN of space from its centre to the
  cones, the same on both sides and everywhere on the lap.
- The ground is flat.
"""
import math

CAR_HALF_WIDTH = 0.70   # m, half the overall width of a typical FS car
CONE_MARGIN = 0.15      # m, gap the driver leaves to the cones
STEP = 1.5              # m, spacing of the points while the line is found
OUT_STEP = 0.5          # m, spacing of the finished track
PASSES = 60             # most sweeps allowed at each bump length
LENGTH_WEIGHT = 0.004   # 1/m^2, how much extra distance counts against a straighter line
SMOOTH_M = 1.0          # m, final smoothing of the line
MIN_LENGTH = 30.0       # m, shortest loop accepted


def _resample(x, y, step, extra=None):
    """Evenly spaced points round a closed loop. extra: values carried along."""
    n = len(x)
    s = [0.0]
    for i in range(n):
        j = (i + 1) % n
        s.append(s[-1] + math.hypot(x[j] - x[i], y[j] - y[i]))
    total = s[-1]
    m = max(8, int(round(total / step)))
    d = total / m
    ox, oy, oe = [], [], []
    i = 0
    for q in range(m):
        t = q * d
        while s[i + 1] < t:
            i += 1
        seg = s[i + 1] - s[i]
        f = (t - s[i]) / seg if seg > 0 else 0.0
        j = (i + 1) % n
        ox.append(x[i] + f * (x[j] - x[i])); oy.append(y[i] + f * (y[j] - y[i]))
        if extra is not None:
            oe.append(extra[i] + f * (extra[j] - extra[i]))
    return ox, oy, d, oe


def _smooth(a, sigma_pts):
    """Gaussian smoothing round a closed loop."""
    if sigma_pts <= 0:
        return list(a)
    r = int(3 * sigma_pts) + 1
    w = [math.exp(-0.5 * (k / sigma_pts) ** 2) for k in range(-r, r + 1)]
    tot = sum(w)
    n = len(a)
    return [sum(w[k + r] * a[(i + k) % n] for k in range(-r, r + 1)) / tot for i in range(n)]


def _curvature(x, y, d):
    n = len(x)
    k = []
    for i in range(n):
        a, b = (i - 1) % n, (i + 1) % n
        dx, dy = (x[b] - x[a]) / (2 * d), (y[b] - y[a]) / (2 * d)
        ddx, ddy = (x[b] - 2 * x[i] + x[a]) / d ** 2, (y[b] - 2 * y[i] + y[a]) / d ** 2
        k.append((dx * ddy - dy * ddx) / max(1e-9, (dx * dx + dy * dy) ** 1.5))
    return k


def driven_line(cx, cy, half, passes=PASSES):
    """The least-bent line inside the lane. cx, cy: lane middle (closed loop);
    half: half lane width at each point, m. Returns x, y, spacing."""
    cx, cy, d, half = _resample(cx, cy, STEP, half)
    n = len(cx)
    nx, ny = [], []
    for i in range(n):
        a, b = (i - 1) % n, (i + 1) % n
        tx, ty = cx[b] - cx[a], cy[b] - cy[a]
        L = math.hypot(tx, ty) or 1.0
        nx.append(-ty / L); ny.append(tx / L)          # points to the left of travel
    room = [max(0.0, h - CAR_HALF_WIDTH - CONE_MARGIN) for h in half]
    e = [0.0] * n                              # how far each point sits to the left of the lane middle

    def bend(i, ei):
        """Rough time to pass point i: the distance it covers divided by the
        speed its bend allows (cornering at LINE_G, never above LINE_VMAX).
        The bend comes from the circle through the point and its neighbours."""
        a, b = (i - 1) % n, (i + 1) % n
        x0, y0 = cx[a] + e[a] * nx[a], cy[a] + e[a] * ny[a]
        x1, y1 = cx[i] + ei * nx[i], cy[i] + ei * ny[i]
        x2, y2 = cx[b] + e[b] * nx[b], cy[b] + e[b] * ny[b]
        la, lb, lc = math.hypot(x1 - x0, y1 - y0), math.hypot(x2 - x1, y2 - y1), math.hypot(x2 - x0, y2 - y0)
        den = la * lb * lc
        kap = 2 * abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)) / den if den > 1e-9 else 10.0
        return 0.5 * (la + lb) * (kap * kap + LENGTH_WEIGHT)

    def span_cost(i, w):
        return sum(bend(j % n, e[j % n]) for j in range(i - w - 1, i + w + 2))

    # Moving one point on its own only ever adds a kink, so whole stretches of
    # the line are nudged together: a gentle bump centred on each point in
    # turn, first long bumps and big steps, then shorter and smaller.
    for w in (8, 4, 2, 1):
        w = min(w, max(1, n // 6))
        shape = [1.0 - abs(q) / (w + 1.0) for q in range(-w, w + 1)]
        step = 0.4
        for _ in range(passes):
            moved = False
            for i in range(n):
                idx = [(i + q) % n for q in range(-w, w + 1)]
                old = [e[j] for j in idx]
                best, cb = None, span_cost(i, w)
                for sgn in (-step, step):
                    for j, o, sh in zip(idx, old, shape):
                        e[j] = max(-room[j], min(room[j], o + sgn * sh))
                    c = span_cost(i, w)
                    if c < cb * (1 - 1e-9):
                        best, cb = [e[j] for j in idx], c
                for j, o in zip(idx, best or old):
                    e[j] = o
                moved = moved or best is not None
            if not moved:
                step /= 2
                if step < 0.02:
                    break
    px = [cx[i] + e[i] * nx[i] for i in range(n)]
    py = [cy[i] + e[i] * ny[i] for i in range(n)]
    return px, py, d


def build_track(cx, cy, half, tid, name, source="", laps=1, cones=None, racing_line=True):
    """A finished track for the solver from the middle of a coned lane.

    cx, cy: lane middle, a closed loop, first point = start line, in driving
    order. half: half lane width, one number or one per point.
    """
    n = len(cx)
    if n < 4:
        raise ValueError("A layout needs at least four points.")
    if not isinstance(half, (list, tuple)):
        half = [float(half)] * n
    x0, y0 = cx[0], cy[0]
    if racing_line:
        x, y, _ = driven_line(cx, cy, list(half))
    else:
        x, y = list(cx), list(cy)
    x, y, d, _ = _resample(x, y, OUT_STEP)
    length = d * len(x)
    if length < MIN_LENGTH:
        raise ValueError("This layout is under %d m long. Draw a longer loop." % MIN_LENGTH)
    x, y = _smooth(x, SMOOTH_M / d), _smooth(y, SMOOTH_M / d)
    x, y, d, _ = _resample(x, y, OUT_STEP)
    # start the lap at the point nearest the start line
    s0 = min(range(len(x)), key=lambda i: (x[i] - x0) ** 2 + (y[i] - y0) ** 2)
    x, y = x[s0:] + x[:s0], y[s0:] + y[:s0]
    k = _smooth(_curvature(x, y, d), 1.0)
    t = dict(id=tid, name=name, kind="endurance" if laps > 1 else "autocross", closed=True,
             ds=round(d, 5), length=round(d * len(x), 2),
             x=[round(v, 2) for v in x], y=[round(v, 2) for v in y], k=[round(v, 5) for v in k],
             laps=laps, standing_start=True, corner_radius=45, source=source)
    if cones:
        t["cones"] = cones
    return t
