"""Tyre force curves: the Pacejka "Magic Formula".

A tyre makes sideways force only when it points slightly away from the way it
is travelling (the slip angle), and makes drive or brake force only when it
turns slightly faster or slower than the road passing under it (the slip
ratio). Force rises with slip, reaches a peak, then falls away.

The Magic Formula is the standard curve for that shape:

    force / peak force = sin(C * atan(B*s - E*(B*s - atan(B*s))))

    s  slip (angle in radians, or ratio)
    B  stiffness factor: how quickly force builds with slip
    C  shape factor: how sharp the peak is and how far force drops after it
    E  curvature factor: how rounded the curve is near the peak

The peak force itself (grip coefficient x load, with load sensitivity) is
handled in vehicle.py. This module supplies the shape, so the simulator can
say how much slip a tyre needs for a given force. That gives slip angles,
steering angle, cornering drag and tyre heating.

Assumptions
- C and E are fixed at typical values for a racing slick; B is set from the
  slip at which the peak occurs, which the user can change.
- The slip at peak does not change with load.
- Combined cornering and braking uses the friction ellipse for the force
  limit, and the pure-slip curves for the slip itself.
"""
import math

C_LAT, E_LAT = 1.5, -0.5
C_LONG, E_LONG = 1.6, -0.3
_N = 64


class MagicCurve:
    def __init__(self, c, e, peak_slip):
        self.c, self.e = c, e
        self.peak_slip = peak_slip
        target = math.tan(math.pi / (2 * c))            # where the sine reaches 1
        self.b = self._solve(target) / peak_slip
        # table of slip needed for each force level, 0 to 100 % of the peak
        self.slip = [self._solve(math.tan(math.asin(i / _N) / c)) / self.b for i in range(_N)] + [peak_slip]
        self.sin_slip = [math.sin(s) for s in self.slip]

    def _solve(self, phi):
        """Find x with x - E*(x - atan x) = phi (the function rises steadily for E < 1)."""
        lo, hi = 0.0, max(1.0, 4 * phi)
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if mid - self.e * (mid - math.atan(mid)) < phi:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def force_ratio(self, s):
        """Force as a share of peak force at slip s."""
        x = self.b * s
        return math.sin(self.c * math.atan(x - self.e * (x - math.atan(x))))

    def slip_for(self, u):
        """Slip needed to make a share u (0 to 1) of the peak force."""
        if u >= 1.0:
            return self.peak_slip
        if u <= 0.0:
            return 0.0
        f = u * _N
        i = int(f)
        return self.slip[i] + (self.slip[i + 1] - self.slip[i]) * (f - i)
