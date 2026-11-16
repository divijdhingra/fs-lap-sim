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

Tyre temperature and wear (see the functions at the bottom)
- Each axle's pair of tyres is one lump with a single temperature. It is
  heated by a share of the sliding and rolling losses and cooled by the air.
- Grip is best at the optimum temperature and falls away either side.
- Wear grows with the energy dissipated by sliding, faster when too hot.
- The thermal constants below are assumed placeholders, chosen so the default
  car settles at about 65 to 70 C in endurance.
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


# ---- temperature and wear (assumed constants, see the notes at the top) ----
HEAT_SHARE = 0.5        # share of the sliding and rolling losses that heats the tyre (rest heats the road)
THERMAL_MASS = 4000.0   # J/K for the two tyres on one axle (tread and part of the carcass)
COOL_0 = 5.0            # W/K lost to the air and road at rest, per axle
COOL_V = 1.0            # extra W/K for each m/s of speed
TEMP_WINDOW = 40.0      # C away from the optimum at which TEMP_LOSS of the grip is gone
TEMP_LOSS = 0.10
TEMP_FLOOR = 0.60       # grip never falls below this share because of temperature alone
WEAR_GRIP_LOSS = 0.12   # grip lost at 100 % wear
WEAR_REF_J_PER_KM = 105000.0   # sliding energy per axle per km at which wear equals the entered wear rate
WEAR_HOT = 0.02         # extra wear for each C above the optimum temperature


def temp_grip(temp, opt):
    """Share of full grip available at a tyre temperature (1.0 at the optimum)."""
    x = (temp - opt) / TEMP_WINDOW
    return max(TEMP_FLOOR, 1.0 - TEMP_LOSS * x * x)


def temp_step(temp, heat_w, v, air, dt):
    """New temperature of one axle's tyres after dt seconds."""
    cool = COOL_0 + COOL_V * v
    # exact answer for constant heating over the step, so long steps cannot overshoot
    target = air + HEAT_SHARE * heat_w / cool
    return target + (temp - target) * math.exp(-cool * dt / THERMAL_MASS)


def wear_added(rate_per_km, slide_j, temp_mean, opt):
    """Wear (% of tyre life) added by slide_j joules of sliding on one axle."""
    hot = 1.0 + WEAR_HOT * max(0.0, temp_mean - opt)
    return rate_per_km * slide_j / WEAR_REF_J_PER_KM * hot


# ---- camber (assumed constants) ----
CAMBER_OPT = -1.5      # deg, lean (top of the tyre towards the inside of the corner) at which cornering grip is best
CAMBER_KY = 0.006      # share of cornering grip lost per deg^2 away from that
CAMBER_KX = 0.004      # share of drive and brake grip lost per deg^2 of lean either way (best upright)
CAMBER_FLOOR = 0.7
