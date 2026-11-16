"""The car model: tyres, aero, load transfer, motor, battery and brakes.

Modelling approach: "quasi-steady-state". At every point on the track the car
is assumed to be in balance for its current speed, cornering force and
acceleration. Fast transients (the car pitching and rolling on its springs
and dampers as the driver turns in or hits a bump) are not simulated.
"""
import math
from .bumps import tyre_load_rms
from .tyre import MagicCurve, C_LAT, E_LAT, C_LONG, E_LONG, WEAR_GRIP_LOSS, temp_grip

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
        self.skill = min(1.0, max(0.3, cond.get("driver", 100) / 100.0))
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
        p_batt = self.i_max * (self.voc - self.i_max * self.r_int)
        self.p_elec = max(0.0, min(self.p_cap, p_batt)) if soc > SOC_MIN else 0.0
        # Motor top speed scales with available voltage.
        w_max = self.rpm_nom * 2 * math.pi / 60.0 * self.voc / self.v_nom
        self.v_top = w_max * self.r / self.gr
        self.f_torque = self.t_peak * self.gr / self.r      # wheel force at peak torque
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
        roll_m = m * ay * h                                 # lateral load transfer moment
        df = roll_m * self.rsd / self.tf
        dr = roll_m * (1 - self.rsd) / self.tr
        df = max(-fzf / 2, min(fzf / 2, df))               # a tyre cannot carry negative load
        dr = max(-fzr / 2, min(fzr / 2, dr))
        # turning left (ay > 0) loads the right-hand tyres
        fl, fr, rl, rr = fzf / 2 - df, fzf / 2 + df, fzr / 2 - dr, fzr / 2 + dr
        if d is not None:
            # d = how far each tyre's load is from its settled value at this point on
            # the track, because the body is still moving on its springs and dampers
            fl, fr, rl, rr = max(0.0, fl + d[0]), max(0.0, fr + d[1]), max(0.0, rl + d[2]), max(0.0, rr + d[3])
        return fl, fr, rl, rr

    def _axle_caps(self, v, ax, ay, d=None):
        """Grip available on each axle: (Fy front, Fy rear, Fx front, Fx rear) in N."""
        fl, fr, rl, rr = self.wheel_loads(v, ax, ay, d)
        ls, fz0 = self.ls, self.fz0
        rv = math.sqrt(v)
        lost = (self.bump_f * rv, self.bump_f * rv, self.bump_r * rv, self.bump_r * rv)
        # grip coefficient from the average load, applied to the load left after bump flutter
        e = [max(0.3 * fz, fz - b) * max(0.4, 1.0 - ls * (fz / fz0 - 1.0))
             for fz, b in zip((fl, fr, rl, rr), lost)]
        ef, er = (e[0] + e[1]) * self.grip_f, (e[2] + e[3]) * self.grip_r
        return self.mu_y0 * ef, self.mu_y0 * er, self.mu_x0 * ef, self.mu_x0 * er

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
        roll_m = m * ay * self.h
        df = roll_m * self.rsd / self.tf
        dr = roll_m * (1 - self.rsd) / self.tr
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
        ef = (fl * e1 + fr * e2) * self.gdf
        er = (rl * e3 + rr * e4) * self.gdr
        may = m * abs(ay)
        uf = may * self.wf / (self.mu_y0 * ef) if ef > 0 else 9.0
        ur = may * (1 - self.wf) / (self.mu_y0 * er) if er > 0 else 9.0
        ok = uf <= 1.0 and ur <= 1.0
        xf = self.mu_x0 * ef * math.sqrt(1 - uf * uf) if uf < 1.0 else 0.0
        xr = self.mu_x0 * er * math.sqrt(1 - ur * ur) if ur < 1.0 else 0.0
        # Cornering drag: a tyre running at a slip angle pushes back along the
        # direction of travel with (sideways force) x sin(slip angle).
        if may > 0.0:
            sk, tab = self.skill * 64, self.lat.sin_slip
            i = int(uf * sk); j = int(ur * sk)
            drag = may * (self.wf * tab[i if i < 64 else 64] + (1 - self.wf) * tab[j if j < 64 else 64])
        else:
            drag = 0.0
        return xf, xr, ok, drag

    def resist(self, v):
        return self.q_cd * v * v + self.rr

    def _traction(self, xf, xr):
        d = self.drive
        return xr if d == "RWD" else (xf + xr if d == "AWD" else xf)

    def ax_max(self, v, kappa, d=None):
        """Best forward acceleration (m/s²) at speed v on a curve of curvature kappa.
        Returns (ax, feasible). feasible is False if the car cannot hold the
        corner at this speed at all."""
        ay = v * v * kappa
        fm = self.motor_force(v)
        res = self.resist(v)
        m = self.m
        ax = 0.0
        for _ in range(12):            # load transfer depends on ax, so iterate to settle
            xf, xr, ok, drag = self._fx_left(v, ax, ay, d)
            new = (min(self._traction(xf, xr), fm) - res - drag) / m
            if abs(new - ax) < 0.04:
                ax = new
                break
            ax = 0.5 * (ax + new)
        if ok or ay == 0.0:
            return ax, True
        # Accelerating shifts load off the front tyres. If that leaves the front
        # unable to hold the corner, ease off until it can.
        xf, xr, ok0, drag = self._fx_left(v, 0.0, ay, d)
        coast = -(res + drag) / m
        if ok0 and min(self._traction(xf, xr), fm) >= res + drag:
            lo = 0.0                       # can at least hold this speed
        elif self._fx_left(v, coast, ay, d)[2]:
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

    def ax_min(self, v, kappa, d=None):
        """Hardest braking (negative, m/s²) at speed v on a curve of curvature kappa."""
        ay = v * v * kappa
        res = self.resist(v)
        ax = -10.0
        b = self.bias
        for _ in range(12):
            xf, xr, _ok, drag = self._fx_left(v, ax, ay, d)
            fb = min(xf / b, xr / (1 - b))
            new = -(fb + res + drag) / self.m
            if abs(new - ax) < 0.04:
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
        cyf, cyr, cxf, cxr = self._axle_caps(v, ax, ay, d)
        fyf, fyr = m * ay * self.wf, m * ay * (1 - self.wf)
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
                    roll=m * ay * self.h / self.k_roll,
                    slip_f=sgn * math.degrees(sa_f), slip_r=sgn * math.degrees(sa_r),
                    steer=math.degrees(self.L * kappa) + sgn * math.degrees(sa_f - sa_r),
                    slip_ratio=100 * max(sr_f, sr_r) * (1 if f_net >= 0 else -1),
                    slide_f=sl_f, slide_r=sl_r, heat_f=sl_f + rr_f, heat_r=sl_r + rr_r)
