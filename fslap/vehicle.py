"""The car model: tyres, aero, load transfer, motor, battery and brakes.

Modelling approach: "quasi-steady-state". At every point on the track the car
is assumed to be in balance for its current speed, cornering force and
acceleration. Fast transients (the car pitching and rolling on its springs
and dampers as the driver turns in or hits a bump) are not simulated.
"""
import math
from .bumps import tyre_load_rms
from .thermal import batt_thermal_mass, batt_derate, motor_derate
from .tyre import MagicCurve, C_LAT, E_LAT, C_LONG, E_LONG, WEAR_GRIP_LOSS, temp_grip, CAMBER_OPT, CAMBER_KY, CAMBER_KX, CAMBER_FLOOR

G = 9.81
SOC_MIN = 0.05             # battery treated as empty below this
# Open-circuit voltage of one Li-ion cell against state of charge (typical NMC cell).
_OCV = [(0.0, 3.00), (0.05, 3.30), (0.10, 3.45), (0.20, 3.55), (0.30, 3.60), (0.50, 3.70),
        (0.70, 3.85), (0.80, 3.95), (0.90, 4.05), (1.0, 4.20)]
CELL_NOMINAL_V = 3.7


def cell_ocv(soc):
    soc = min(1.0, max(0.0, soc))
    for (s0, v0), (s1, v1) in zip(_OCV, _OCV[1:]):
        if soc <= s1:
            return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    return _OCV[-1][1]


TC_SHARE = 0.97          # share of available traction used with traction control on (assumption)


class Vehicle:
    def __init__(self, car, cond):
        c = car
        self.m = c["mass_car"] + c["mass_driver"]
        self.L = c["wheelbase"]
        self.tf, self.tr = c["track_f"], c["track_r"]
        self.h = c["cg_height"]
        self.wf = c["weight_front"] / 100.0
        self.mu_x0, self.mu_y0 = c["mu_x"], c["mu_y"]
        self.fz0, self.ls = c["tyre_ref_load"], c["load_sens"]
        self.lat = MagicCurve(C_LAT, E_LAT, math.radians(c["peak_slip_angle"]))    # cornering force curve
        self.lon = MagicCurve(C_LONG, E_LONG, c["peak_slip_ratio"] / 100.0)        # drive / brake force curve
        self.r = c["tyre_radius"]
        self.crr = c["rolling_res"]
        self.rho = 1.225 * 288.15 / (273.15 + cond["air_temp"])
        self.q_cl = 0.5 * self.rho * c["cl_a"]
        self.q_cd = 0.5 * self.rho * c["cd_a"]
        self.ab = c["aero_balance"] / 100.0
        self.drive = c["drive"]
        diff = c.get("diff", "Limited slip")
        self.open_diff = diff == "Open"
        # torque vectoring: largest sideways force it can move between axles (N) = turning moment / wheelbase
        self.tv_force = (c.get("tv_moment", 0.0) / c["wheelbase"]) if diff == "Torque vectoring" else 0.0
        self._of = self._or = 1.0           # open differential: share of an axle's traction that is usable
        self.hint_a = self.hint_b = None    # last answers at each track point, used as starting guesses
        self.t_peak = c["motor_torque"]
        self.p_motor = c["motor_power"] * 1000.0
        self.rpm_nom = c["motor_rpm"]
        self.gr = c["gear_ratio"]
        self.eff = c["drive_eff"] / 100.0
        self.ns = c["cells_series"]
        self.v_nom = self.ns * CELL_NOMINAL_V
        self.cap_ah = c["batt_kwh"] * 1000.0 / self.v_nom
        self.r_int = c["batt_res"]
        self.i_max = c["batt_imax"]
        self.regen_w = c["regen_kw"] * 1000.0
        # Heat: thermal mass and cooling of the pack and of the motor with its inverter (see thermal.py)
        self.batt_mass_jk = batt_thermal_mass(c["batt_kwh"])
        self.batt_cool, self.motor_cool = c.get("batt_cool", 15.0), c.get("motor_cool", 30.0)
        self.batt_temp = cond.get("batt_temp", 25.0)        # temperatures now (they move during a lap)
        self.motor_temp = cond.get("motor_temp", 40.0)
        self.bias = c["brake_bias"] / 100.0
        # Springs, anti-roll bars and dampers, all expressed at the wheel.
        self.kw_f, self.kw_r = c["wheel_rate_f"] * 1000.0, c["wheel_rate_r"] * 1000.0     # N/m
        self.c_f, self.c_r = c["damper_f"], c["damper_r"]                                # N·s/m
        self.m_unsprung = c["unsprung"]
        self.m_sprung = max(20.0, self.m - 4 * self.m_unsprung)
        # Roll stiffness of an axle (N·m/rad): two springs half a track width from
        # the centre line, plus the anti-roll bar.
        self.kroll_f = self.kw_f * self.tf ** 2 / 2 + math.degrees(c["arb_f"])
        self.kroll_r = self.kw_r * self.tr ** 2 / 2 + math.degrees(c["arb_r"])
        self.croll_f = self.c_f * self.tf ** 2 / 2          # roll damping, N·m·s/rad
        self.croll_r = self.c_r * self.tr ** 2 / 2
        self.k_roll = math.radians(self.kroll_f + self.kroll_r)      # N·m/deg, both axles
        self.rsd = self.kroll_f / (self.kroll_f + self.kroll_r)     # roll stiffness distribution, front share
        # Suspension geometry. The roll centre of an axle is the point the body
        # rolls about at that axle. Cornering force acting at roll centre height
        # goes straight through the suspension links into the tyres; only the
        # rest (CoG height above the roll axis) rolls the body on its springs.
        self.rc_f, self.rc_r = c.get("rc_f", 0.0), c.get("rc_r", 0.0)
        self.h_roll = self.h - (self.wf * self.rc_f + (1 - self.wf) * self.rc_r)
        # lateral load transfer on each axle per m/s² of cornering (N per m/s²)
        self.lt_f = self.m * (self.wf * self.rc_f + self.h_roll * self.rsd) / self.tf
        self.lt_r = self.m * ((1 - self.wf) * self.rc_r + self.h_roll * (1 - self.rsd)) / self.tr
        self.roll_per_ay = self.m * self.h_roll / self.k_roll        # deg of roll per m/s²
        self.anti_dive = min(1.0, max(0.0, c.get("anti_dive", 0.0) / 100.0))
        self.anti_squat = min(1.0, max(0.0, c.get("anti_squat", 0.0) / 100.0))
        # Camber: how far the tyres lean. Negative = tops lean in towards the car.
        self.cam_f, self.cam_r = c.get("camber_f", 0.0), c.get("camber_r", 0.0)
        # body roll tips the tyres over; camber gain in the linkage gives part of it back
        self.cam_roll = self.roll_per_ay * (1.0 - min(1.0, max(0.0, c.get("camber_gain", 0.0))))
        # Road bumps: tyre load flutter grows with the square root of speed, so work
        # it out once at 1 m/s for a front and a rear corner (see bumps.py).
        self.kt = c["tyre_kv"] * 1000.0
        self.gd0 = cond.get("roughness", 0) * 1e-6
        self.ms_f = max(5.0, self.m * self.wf / 2 - self.m_unsprung)        # sprung mass on one front corner
        self.ms_r = max(5.0, self.m * (1 - self.wf) / 2 - self.m_unsprung)
        self.sig_f = tyre_load_rms(self.ms_f, self.m_unsprung, self.kw_f, self.c_f, self.kt, 1.0, self.gd0)
        self.sig_r = tyre_load_rms(self.ms_r, self.m_unsprung, self.kw_r, self.c_r, self.kt, 1.0, self.gd0)
        self.bump_f = c["bump_sens"] * self.sig_f     # grip-equivalent load lost per sqrt(m/s)
        self.bump_r = c["bump_sens"] * self.sig_r
        self.p_cap = cond["power_cap"] * 1000.0
        self.track_grip = cond["track_grip"] / 100.0
        # Driver: share of the tyres' grip used for cornering, and separately for
        # braking and for putting power down (see the notes in solver.py).
        clip = lambda x: min(1.0, max(0.3, x / 100.0))
        self.skill = clip(cond.get("driver", 100))
        self.k_brake = clip(cond.get("driver_brake", 100)) / self.skill
        power = clip(cond.get("driver_power", 100))
        if cond.get("traction_control", "Off") == "On":
            power = max(power, TC_SHARE)        # the controller holds the tyres near their best slip
        self.k_power = power / self.skill
        self.lift_m = max(0.0, cond.get("lift_coast", 0.0))     # lifts off this far before braking
        self.rr = self.crr * self.m * G
        self.air_temp = cond["air_temp"]
        self.t_opt = c.get("tyre_temp_opt", 70.0)
        self.wear_rate = cond.get("wear_rate", 0.0)
        t0 = cond.get("tyre_temp", self.t_opt)
        self.temp_f = self.temp_r = t0          # tyre temperatures now (they move during a lap)
        self.set_state(cond["soc_start"] / 100.0)
        self.set_tyres(cond["tyre_wear"], cond["tyre_wear"], t0, t0)

    # ---- things that change during a run -------------------------------
    def set_tyres(self, wear_f, wear_r, tg_f=None, tg_r=None):
        """Update tyre wear (%) and the tyre temperatures used for grip (C), per axle."""
        self.wear_f, self.wear_r = min(100.0, wear_f), min(100.0, wear_r)
        self.wear = 0.5 * (self.wear_f + self.wear_r)
        if tg_f is not None:
            self.tg_f, self.tg_r = tg_f, tg_r
        # grip of each axle's tyres: track surface x wear x temperature
        self.grip_f = self.track_grip * (1.0 - WEAR_GRIP_LOSS * self.wear_f / 100.0) * temp_grip(self.tg_f, self.t_opt)
        self.grip_r = self.track_grip * (1.0 - WEAR_GRIP_LOSS * self.wear_r / 100.0) * temp_grip(self.tg_r, self.t_opt)
        # The grip the driver is prepared to use. The tyres still have grip_f and
        # grip_r; an imperfect driver simply leaves some of it unused.
        self.gdf, self.gdr = self.grip_f * self.skill, self.grip_r * self.skill

    def set_state(self, soc):
        """Update battery charge; recompute what depends on it."""
        self.soc = soc
        self.voc = self.ns * cell_ocv(soc)
        # Most power the battery can give at its terminals: current limit, with voltage sag.
        # A hot pack has its current limit cut back; a hot motor its torque.
        self.batt_cut, self.motor_cut = batt_derate(self.batt_temp), motor_derate(self.motor_temp)
        i_max = self.i_max * self.batt_cut
        p_batt = i_max * (self.voc - i_max * self.r_int)
        self.p_elec = max(0.0, min(self.p_cap, p_batt)) if soc > SOC_MIN else 0.0
        # Motor top speed scales with available voltage.
        w_max = self.rpm_nom * 2 * math.pi / 60.0 * self.voc / self.v_nom
        self.v_top = w_max * self.r / self.gr
        self.f_torque = self.t_peak * self.motor_cut * self.gr / self.r      # wheel force at peak torque
        self.p_wheel = min(self.p_motor, self.p_elec * self.eff)

    # ---- forces ---------------------------------------------------------
    def motor_force(self, v):
        """Most push the powertrain can deliver at the wheels at speed v (N)."""
        if v >= self.v_top:
            return 0.0
        return min(self.f_torque, self.p_wheel / max(v, 0.5))

    def wheel_loads(self, v, ax, ay, d=None):
        """Vertical load on each tyre (N): FL, FR, RL, RR. ay > 0 means turning left."""
        m, h = self.m, self.h
        down = self.q_cl * v * v
        long_t = m * ax * h / self.L                       # longitudinal load transfer
        fzf = max(0.0, m * G * self.wf + down * self.ab - long_t)
        fzr = max(0.0, m * G * (1 - self.wf) + down * (1 - self.ab) + long_t)
        df = ay * self.lt_f                                 # lateral load transfer, each axle
        dr = ay * self.lt_r
        df = max(-fzf / 2, min(fzf / 2, df))               # a tyre cannot carry negative load
        dr = max(-fzr / 2, min(fzr / 2, dr))
        # turning left (ay > 0) loads the right-hand tyres
        fl, fr, rl, rr = fzf / 2 - df, fzf / 2 + df, fzr / 2 - dr, fzr / 2 + dr
        if d is not None:
            # d = how far each tyre's load is from its settled value at this point on
            # the track, because the body is still moving on its springs and dampers
            fl, fr, rl, rr = max(0.0, fl + d[0]), max(0.0, fr + d[1]), max(0.0, rl + d[2]), max(0.0, rr + d[3])
        return fl, fr, rl, rr

    def _axle_caps(self, v, ax, ay, d=None, fz=None):
        """Grip available on each axle: (Fy front, Fy rear, Fx front, Fx rear) in N."""
        fl, fr, rl, rr = fz if fz is not None else self.wheel_loads(v, ax, ay, d)
        ls, fz0 = self.ls, self.fz0
        rv = math.sqrt(v)
        lost = (self.bump_f * rv, self.bump_f * rv, self.bump_r * rv, self.bump_r * rv)
        # grip coefficient from the average load, applied to the load left after bump flutter
        e = [max(0.3 * fz, fz - b) * max(0.4, 1.0 - ls * (fz / fz0 - 1.0))
             for fz, b in zip((fl, fr, rl, rr), lost)]
        cy, cx = self._camber_factors(ay)
        gf, gr = self.grip_f, self.grip_r
        return (self.mu_y0 * (e[0] * cy[0] + e[1] * cy[1]) * gf, self.mu_y0 * (e[2] * cy[2] + e[3] * cy[3]) * gr,
                self.mu_x0 * (e[0] * cx[0] + e[1] * cx[1]) * gf, self.mu_x0 * (e[2] * cx[2] + e[3] * cx[3]) * gr)

    def _camber_factors(self, ay):
        """Share of grip each tyre keeps at its current lean: (cornering x4, drive/brake x4), FL FR RL RR.

        In a corner the body rolls and tips both tyres' tops towards the outside.
        Static negative camber leans the outside tyre the right way (into the
        corner) and the inside tyre the wrong way."""
        pr = abs(ay) * self.cam_roll
        cy, cx = [], []
        for cam in (self.cam_f, self.cam_r):
            out, ins = cam + pr, -cam + pr            # lean measured towards the outside of the corner
            yo = max(CAMBER_FLOOR, 1 - CAMBER_KY * (out - CAMBER_OPT) ** 2)
            yi = max(CAMBER_FLOOR, 1 - CAMBER_KY * (ins - CAMBER_OPT) ** 2)
            xo = max(CAMBER_FLOOR, 1 - CAMBER_KX * out * out)
            xi = max(CAMBER_FLOOR, 1 - CAMBER_KX * ins * ins)
            if ay >= 0: cy += [yi, yo]; cx += [xi, xo]      # turning left: right-hand tyres are outside
            else: cy += [yo, yi]; cx += [xo, xi]
        return cy, cx

    def _fx_left(self, v, ax, ay, d=None):
        """Longitudinal grip left on each axle after cornering has taken its share
        (friction ellipse). Returns (front, rear, feasible).

        This is the same physics as wheel_loads + _axle_caps, written out in one
        piece because it is called hundreds of thousands of times per lap."""
        m = self.m
        down = self.q_cl * v * v
        long_t = m * ax * self.h / self.L
        fzf = m * G * self.wf + down * self.ab - long_t
        fzr = m * G * (1 - self.wf) + down * (1 - self.ab) + long_t
        if fzf < 0.0: fzf = 0.0
        if fzr < 0.0: fzr = 0.0
        df = ay * self.lt_f
        dr = ay * self.lt_r
        hf, hr = 0.5 * fzf, 0.5 * fzr
        if df > hf: df = hf
        elif df < -hf: df = -hf
        if dr > hr: dr = hr
        elif dr < -hr: dr = -hr
        fl, fr, rl, rr = hf - df, hf + df, hr - dr, hr + dr
        if d is not None:
            fl += d[0]; fr += d[1]; rl += d[2]; rr += d[3]
            if fl < 0.0: fl = 0.0
            if fr < 0.0: fr = 0.0
            if rl < 0.0: rl = 0.0
            if rr < 0.0: rr = 0.0
        ls, inv = self.ls, 1.0 / self.fz0
        c = 1.0 + ls
        e1 = c - ls * fl * inv; e2 = c - ls * fr * inv; e3 = c - ls * rl * inv; e4 = c - ls * rr * inv
        if e1 < 0.4: e1 = 0.4
        if e2 < 0.4: e2 = 0.4
        if e3 < 0.4: e3 = 0.4
        if e4 < 0.4: e4 = 0.4
        bf, br = self.bump_f, self.bump_r
        if bf > 0.0 or br > 0.0:              # bumps: part of each tyre's load is lost to flutter
            rv = math.sqrt(v)
            bf *= rv; br *= rv
            fl = fl - bf if fl - bf > 0.3 * fl else 0.3 * fl
            fr = fr - bf if fr - bf > 0.3 * fr else 0.3 * fr
            rl = rl - br if rl - br > 0.3 * rl else 0.3 * rl
            rr = rr - br if rr - br > 0.3 * rr else 0.3 * rr
        # camber: grip each tyre keeps at its current lean (same sums as _camber_factors)
        pr = (ay if ay >= 0 else -ay) * self.cam_roll
        ky, kx, opt, flo = CAMBER_KY, CAMBER_KX, CAMBER_OPT, CAMBER_FLOOR
        cam = self.cam_f
        go = cam + pr; gi = pr - cam
        yo = 1 - ky * (go - opt) * (go - opt); yi = 1 - ky * (gi - opt) * (gi - opt)
        xo = 1 - kx * go * go; xi = 1 - kx * gi * gi
        if yo < flo: yo = flo
        if yi < flo: yi = flo
        if xo < flo: xo = flo
        if xi < flo: xi = flo
        fl *= e1; fr *= e2; rl *= e3; rr *= e4
        if ay >= 0: efy = fl * yi + fr * yo; efx = fl * xi + fr * xo
        else: efy = fl * yo + fr * yi; efx = fl * xo + fr * xi
        xo_f, xi_f = xo, xi
        cam = self.cam_r
        go = cam + pr; gi = pr - cam
        yo = 1 - ky * (go - opt) * (go - opt); yi = 1 - ky * (gi - opt) * (gi - opt)
        xo = 1 - kx * go * go; xi = 1 - kx * gi * gi
        if yo < flo: yo = flo
        if yi < flo: yi = flo
        if xo < flo: xo = flo
        if xi < flo: xi = flo
        if ay >= 0: ery = rl * yi + rr * yo; erx = rl * xi + rr * xo
        else: ery = rl * yo + rr * yi; erx = rl * xo + rr * xi
        if self.open_diff:
            # an open differential gives both wheels the same torque, so the axle can
            # only put down twice what the weaker tyre (the inside one) can take
            if ay >= 0: a1 = fl * xi_f; a2 = fr * xo_f; a3 = rl * xi; a4 = rr * xo
            else: a1 = fl * xo_f; a2 = fr * xi_f; a3 = rl * xo; a4 = rr * xi
            self._of = 2 * (a1 if a1 < a2 else a2) / efx if efx > 0 else 1.0
            self._or = 2 * (a3 if a3 < a4 else a4) / erx if erx > 0 else 1.0
        gdf, gdr = self.gdf, self.gdr
        may = m * abs(ay)
        capf, capr = self.mu_y0 * efy * gdf, self.mu_y0 * ery * gdr
        fyf, fyr = may * self.wf, may * (1 - self.wf)
        if self.tv_force > 0.0 and may > 0.0:
            fyf, fyr = self._tv_split(fyf, fyr, capf, capr)
        uf = fyf / capf if capf > 0 else 9.0
        ur = fyr / capr if capr > 0 else 9.0
        ok = uf <= 1.0 and ur <= 1.0
        xf = self.mu_x0 * efx * gdf * math.sqrt(1 - uf * uf) if uf < 1.0 else 0.0
        xr = self.mu_x0 * erx * gdr * math.sqrt(1 - ur * ur) if ur < 1.0 else 0.0
        # Cornering drag: a tyre running at a slip angle pushes back along the
        # direction of travel with (sideways force) x sin(slip angle).
        if may > 0.0:
            sk, tab = self.skill * 64, self.lat.sin_slip
            i = int(uf * sk); j = int(ur * sk)
            drag = fyf * tab[i if i < 64 else 64] + fyr * tab[j if j < 64 else 64]
        else:
            drag = 0.0
        return xf, xr, ok, drag

    def resist(self, v):
        return self.q_cd * v * v + self.rr

    def _tv_split(self, fyf, fyr, capf, capr):
        """Torque vectoring: driving the outside wheel harder than the inside one
        turns the car, which takes cornering work off the axle nearer its limit.
        Returns the cornering force each axle is left to supply."""
        tot = capf + capr
        if tot <= 0:
            return fyf, fyr
        s = fyf - (fyf + fyr) * capf / tot          # force to move off the front to level the two axles
        lim = self.tv_force
        s = lim if s > lim else (-lim if s < -lim else s)
        return fyf - s, fyr + s

    def _traction(self, xf, xr):
        d = self.drive
        if self.open_diff:
            xf *= self._of; xr *= self._or
        return self.k_power * (xr if d == "RWD" else (xf + xr if d == "AWD" else xf))

    def ax_max(self, v, kappa, d=None, guess=None):
        """Best forward acceleration (m/s²) at speed v on a curve of curvature kappa.
        Returns (ax, feasible). feasible is False if the car cannot hold the
        corner at this speed at all. guess: the answer from a nearby case (the
        same point on the lap before), which saves most of the search."""
        ay = v * v * kappa
        fm = self.motor_force(v)
        res = self.resist(v)
        m = self.m
        fx, trac = self._fx_left, self._traction
        # Load transfer depends on ax, and grip depends on load transfer, so the
        # answer has to be found by trial: try a value, see what acceleration the
        # car could then make, and step towards where the two agree (secant steps).
        t0 = None
        if guess is None:
            xf, xr, ok, drag = fx(v, 0.0, ay, d)
            t0, ok0, drag0 = trac(xf, xr), ok, drag
            ax = (min(t0, fm) - res - drag) / m
            a0, ga0 = 0.0, ax
            done = abs(ax) < 0.04
        else:
            ax, a0, ga0, done = guess, None, 0.0, False
        if not done:
            for it in range(14):
                xf, xr, ok, drag = fx(v, ax, ay, d)
                new = (min(trac(xf, xr), fm) - res - drag) / m
                if abs(new - ax) < 0.02:
                    ax = new
                    break
                if a0 is None:
                    a0, ga0, ax = ax, new, new                # first step from a guess: just take it
                    continue
                step = ax - a0
                slope = (new - ga0) / step if step != 0.0 else 0.0
                a0, ga0 = ax, new
                if it < 8 and -3.0 < slope < 0.9:
                    ax = ax + (new - ax) / (1.0 - slope)      # secant step
                else:
                    ax = 0.5 * (ax + new)                     # cautious fallback
        if ok or ay == 0.0:
            return ax, True
        # Accelerating shifts load off the front tyres. If that leaves the front
        # unable to hold the corner, ease off until it can.
        if t0 is None:
            xf, xr, ok0, drag0 = fx(v, 0.0, ay, d)
            t0 = trac(xf, xr)
        coast = -(res + drag0) / m
        if ok0 and min(t0, fm) >= res + drag0:
            lo = 0.0                       # can at least hold this speed
        elif fx(v, coast, ay, d)[2]:
            lo = coast
        else:
            return coast, False
        hi = ax
        for _ in range(8):
            mid = 0.5 * (lo + hi)
            xf, xr, ok, drag = self._fx_left(v, mid, ay, d)
            if ok and (min(self._traction(xf, xr), fm) - res - drag) / m >= mid:
                lo = mid
            else:
                hi = mid
        return lo, True

    def ax_min(self, v, kappa, d=None, guess=None):
        """Hardest braking (negative, m/s²) at speed v on a curve of curvature kappa.
        guess: the answer from a nearby case, to start the search from."""
        ay = v * v * kappa
        res = self.resist(v)
        ax = -10.0 if guess is None else guess
        b = self.bias
        for _ in range(12):
            xf, xr, _ok, drag = self._fx_left(v, ax, ay, d)
            fb = self.k_brake * min(xf / b, xr / (1 - b))
            new = -(fb + res + drag) / self.m
            if abs(new - ax) < 0.02:
                ax = new
                break
            ax = 0.5 * (ax + new)
        return ax

    def corner_speed(self, kappa, d=None, guess=None):
        """Highest steady speed (m/s) the car can hold on a curve of curvature kappa.
        guess: a nearby answer to start the search from (saves time)."""
        kappa = abs(kappa)
        top = self.v_top
        if kappa < 1e-5:
            return top
        def ok(v):
            # can hold the corner at constant speed: grip for the cornering force
            # with enough left over on the driven tyres to cancel air and cornering drag
            xf, xr, feas, drag = self._fx_left(v, 0.0, v * v * kappa, d)
            return feas and min(self._traction(xf, xr), self.motor_force(v)) >= self.resist(v) + drag
        lo, hi, n = 0.0, top, 22
        if guess is not None:
            a, b = 0.90 * guess, min(top, 1.08 * guess)
            if ok(a) and not ok(b):
                lo, hi, n = a, b, 9
        if hi >= top and ok(top):
            return top
        for _ in range(n):
            mid = 0.5 * (lo + hi)
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo

    # ---- battery --------------------------------------------------------
    def battery_current(self, p_term):
        """Current (A) for a given terminal power (W). Negative power = charging."""
        disc = self.voc * self.voc - 4 * self.r_int * p_term
        return (self.voc - math.sqrt(max(0.0, disc))) / (2 * self.r_int)

    def point_state(self, v, kappa, ax, d=None):
        """Everything worth showing at one point on the track."""
        m = self.m
        ay = v * v * kappa
        fz = self.wheel_loads(v, ax, ay, d)
        cyf, cyr, cxf, cxr = self._axle_caps(v, ax, ay, d, fz)
        fyf, fyr = m * ay * self.wf, m * ay * (1 - self.wf)
        tv_m = 0.0
        if self.tv_force > 0.0 and ay != 0.0:
            a, b = self._tv_split(abs(fyf), abs(fyr), cyf * self.skill, cyr * self.skill)
            tv_m = (abs(fyf) - a) * self.L * (1.0 if ay >= 0 else -1.0)     # turning moment applied, N·m
            fyf, fyr = math.copysign(a, ay), math.copysign(b, ay)
        # slip angle each axle needs for its cornering force (Magic Formula, see tyre.py)
        sa_f = self.lat.slip_for(abs(fyf) / cyf) if cyf > 0 else 0.0
        sa_r = self.lat.slip_for(abs(fyr) / cyr) if cyr > 0 else 0.0
        drag = abs(fyf) * math.sin(sa_f) + abs(fyr) * math.sin(sa_r)        # cornering drag
        f_net = m * ax + self.resist(v) + drag    # force the tyres must supply along the car
        dr = self.drive
        p_term = 0.0; torque = 0.0; regen = 0.0
        if f_net >= 0:
            if dr == "RWD": fxf, fxr = 0.0, f_net
            elif dr == "FWD": fxf, fxr = f_net, 0.0
            else:
                tot = cxf + cxr
                fxf, fxr = f_net * cxf / tot, f_net * cxr / tot
        else:
            fxf, fxr = f_net * self.bias, f_net * (1 - self.bias)
        # wheel slip each axle needs for its drive or brake force
        sr_f = self.lon.slip_for(abs(fxf) / cxf) if cxf > 0 else 0.0
        sr_r = self.lon.slip_for(abs(fxr) / cxr) if cxr > 0 else 0.0
        if f_net >= 0:
            # driven wheels turn a little faster than the road, which costs extra power
            spin = (abs(fxf) * sr_f + abs(fxr) * sr_r) * v
            p_term = min((f_net * v + spin) / self.eff, self.p_elec)   # clamp step-size rounding
            torque = f_net * self.r / self.gr
        else:
            fb = -f_net
            share = (1 - self.bias) if dr == "RWD" else (self.bias if dr == "FWD" else 1.0)
            if self.regen_w > 0 and v > 1.4 and self.soc < 0.98:      # rules: no regen below 5 km/h
                f_reg = min(fb * share, self.f_torque)
                regen = min(f_reg * v * self.eff, self.regen_w)
                p_term = -regen
                torque = -regen / self.eff / v * self.r / self.gr
        uf = math.hypot(fyf / cyf, fxf / cxf) if cyf > 0 else 0.0
        ur = math.hypot(fyr / cyr, fxr / cxr) if cyr > 0 else 0.0
        sgn = 1.0 if ay >= 0 else -1.0
        # heat generated where each axle's tyres slide against the road (W)
        sl_f = (abs(fyf) * math.sin(sa_f) + abs(fxf) * sr_f) * v
        sl_r = (abs(fyr) * math.sin(sa_r) + abs(fxr) * sr_r) * v
        rr_f, rr_r = self.crr * (fz[0] + fz[1]) * v, self.crr * (fz[2] + fz[3]) * v
        return dict(fz=fz, uf=uf, ur=ur, p=p_term, torque=torque,
                    roll=ay * self.roll_per_ay, tv=tv_m, cam_fo=self.cam_f + abs(ay) * self.cam_roll, cam_ro=self.cam_r + abs(ay) * self.cam_roll,
                    slip_f=sgn * math.degrees(sa_f), slip_r=sgn * math.degrees(sa_r),
                    steer=math.degrees(self.L * kappa) + sgn * math.degrees(sa_f - sa_r),
                    slip_ratio=100 * max(sr_f, sr_r) * (1 if f_net >= 0 else -1),
                    slide_f=sl_f, slide_r=sl_r, heat_f=sl_f + rr_f, heat_r=sl_r + rr_r)
