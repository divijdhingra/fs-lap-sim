"""The g-g-v diagram: the car's grip envelope.

At a given speed, how hard can the car accelerate or brake while it is also
cornering at a given rate? Plot cornering g across and acceleration or braking
g up and down, and the answers trace a closed outline. Inside it the car has
grip to spare; on it the tyres (or the motor) are at their limit. Because
downforce, drag and motor force all change with speed, there is one outline
per speed, hence g-g-v.

It is built from the same three limit functions the lap solver uses, so the
lap can never leave it:
- the hardest cornering the car can hold at that speed without slowing down,
- the best forward acceleration left over at each cornering rate (limited by
  rear tyre grip at low speed, by motor torque and power at high speed),
- the hardest braking at each cornering rate (limited by tyre grip and the
  fixed brake bias).

Assumptions
- The car is in its starting state: battery charge, tyre temperature, tyre
  wear, and battery and motor temperature as set under Conditions.
- Tyre loads are the settled ones (no spring, damper or bump effects).
- The driver's grip shares are included, so a cautious driver shrinks the
  outline. Left and right turns are taken as mirror images.
"""
from .vehicle import Vehicle

G = 9.81
SPEEDS = (10.0, 15.0, 20.0, 25.0, 30.0)     # m/s (36, 54, 72, 90, 108 km/h)


def envelope(veh, v, n=15):
    """Outline at speed v (m/s): lists of cornering g, best acceleration g and
    hardest braking g (negative), from straight-line to maximum cornering."""
    lo, hi = 0.0, 60.0
    for _ in range(24):                       # hardest cornering the car can hold at a steady speed
        mid = 0.5 * (lo + hi)
        xf, xr, feas, drag = veh._fx_left(v, 0.0, mid)
        # same test the solver uses for corner speed: grip to hold the turn, and enough drive left to hold the speed
        if feas and min(veh._traction(xf, xr), veh.motor_force(v)) >= veh.resist(v) + drag:
            lo = mid
        else:
            hi = mid
    ay, acc, brk = [], [], []
    ga = gb = None
    for i in range(n):
        a = lo * (1.0 - (1.0 - i / (n - 1.0)) ** 2)      # more points near the cornering limit, where the outline bends
        kap = a / (v * v)
        ga, _ok = veh.ax_max(v, kap, None, ga)
        gb = veh.ax_min(v, kap, None, gb)
        ay.append(round(a / G, 3)); acc.append(round(ga / G, 3)); brk.append(round(min(gb, ga) / G, 3))
    return dict(v=v, ay=ay, acc=acc, brake=brk)


def ggv(car, cond):
    veh = Vehicle(car, cond)
    speeds = [v for v in SPEEDS if v < 0.97 * veh.v_top]
    return dict(speeds=speeds, v_top=round(veh.v_top, 2), curves=[envelope(veh, v) for v in speeds])
