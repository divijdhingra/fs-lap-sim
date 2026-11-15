"""The car model: tyres, aero, load transfer, motor, battery and brakes.

Modelling approach: "quasi-steady-state". At every point on the track the car
is assumed to be in balance for its current speed, cornering force and
acceleration. Fast transients (the car pitching and rolling on its springs
and dampers as the driver turns in or hits a bump) are not simulated.
"""
import math

G = 9.81
WEAR_GRIP_LOSS = 0.12      # grip lost at 100 % tyre wear (assumption)
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
        self.p_cap = cond["power_cap"] * 1000.0
        self.track_grip = cond["track_grip"] / 100.0
        self.rr = self.crr * self.m * G
        self.set_state(cond["soc_start"] / 100.0, cond["tyre_wear"])

    # ---- things that change during a run -------------------------------
    def set_state(self, soc, wear_pct):
        """Update battery charge and tyre wear; recompute what depends on them."""
        self.soc = soc
        self.wear = min(100.0, wear_pct)
        self.grip = self.track_grip * (1.0 - WEAR_GRIP_LOSS * self.wear / 100.0)
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

    def wheel_loads(self, v, ax, ay):
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
        return fzf / 2 - df, fzf / 2 + df, fzr / 2 - dr, fzr / 2 + dr

    def _axle_caps(self, v, ax, ay):
        """Grip available on each axle: (Fy front, Fy rear, Fx front, Fx rear) in N."""
        fl, fr, rl, rr = self.wheel_loads(v, ax, ay)
        ls, fz0, g = self.ls, self.fz0, self.grip
        e = [fz * max(0.4, 1.0 - ls * (fz / fz0 - 1.0)) * g for fz in (fl, fr, rl, rr)]
        ef, er = e[0] + e[1], e[2] + e[3]
        return self.mu_y0 * ef, self.mu_y0 * er, self.mu_x0 * ef, self.mu_x0 * er

    def _fx_left(self, v, ax, ay):
        """Longitudinal grip left on each axle after cornering has taken its share
        (friction ellipse). Returns (front, rear, feasible)."""
        cyf, cyr, cxf, cxr = self._axle_caps(v, ax, ay)
        may = self.m * abs(ay)
        uf = may * self.wf / cyf if cyf > 0 else 9.0
        ur = may * (1 - self.wf) / cyr if cyr > 0 else 9.0
        ok = uf <= 1.0 and ur <= 1.0
        return (cxf * math.sqrt(max(0.0, 1 - uf * uf)), cxr * math.sqrt(max(0.0, 1 - ur * ur)), ok)

    def resist(self, v):
        return self.q_cd * v * v + self.rr

    def _traction(self, xf, xr):
        d = self.drive
        return xr if d == "RWD" else (xf + xr if d == "AWD" else xf)

    def ax_max(self, v, kappa):
        """Best forward acceleration (m/s²) at speed v on a curve of curvature kappa.
        Returns (ax, feasible). feasible is False if the car cannot hold the
        corner at this speed at all."""
        ay = v * v * kappa
        fm = self.motor_force(v)
        res = self.resist(v)
        m = self.m
        ax = 0.0
        for _ in range(12):            # load transfer depends on ax, so iterate to settle
            xf, xr, ok = self._fx_left(v, ax, ay)
            new = (min(self._traction(xf, xr), fm) - res) / m
            if abs(new - ax) < 0.01:
                ax = new
                break
            ax = 0.5 * (ax + new)
        if ok or ay == 0.0:
            return ax, True
        # Accelerating shifts load off the front tyres. If that leaves the front
        # unable to hold the corner, ease off until it can.
        coast = -res / m
        xf, xr, ok0 = self._fx_left(v, 0.0, ay)
        if ok0 and min(self._traction(xf, xr), fm) >= res:
            lo = 0.0                       # can at least hold this speed
        elif self._fx_left(v, coast, ay)[2]:
            lo = coast
        else:
            return coast, False
        hi = ax
        for _ in range(8):
            mid = 0.5 * (lo + hi)
            xf, xr, ok = self._fx_left(v, mid, ay)
            if ok and (min(self._traction(xf, xr), fm) - res) / m >= mid:
                lo = mid
            else:
                hi = mid
        return lo, True

    def ax_min(self, v, kappa):
        """Hardest braking (negative, m/s²) at speed v on a curve of curvature kappa."""
        ay = v * v * kappa
        res = self.resist(v)
        ax = -10.0
        b = self.bias
        for _ in range(12):
            xf, xr, _ok = self._fx_left(v, ax, ay)
            fb = min(xf / b, xr / (1 - b))
            new = -(fb + res) / self.m
            if abs(new - ax) < 0.01:
                ax = new
                break
            ax = 0.5 * (ax + new)
        return ax

    def corner_speed(self, kappa):
        """Highest steady speed (m/s) the car can hold on a curve of curvature kappa."""
        kappa = abs(kappa)
        hi = self.v_top
        if kappa < 1e-5:
            return hi
        def ok(v):
            # can hold the corner at constant speed: grip for the cornering force
            # with enough left over on the driven tyres to cancel drag
            xf, xr, feas = self._fx_left(v, 0.0, v * v * kappa)
            return feas and min(self._traction(xf, xr), self.motor_force(v)) >= self.resist(v)
        if ok(hi):
            return hi
        lo = 0.0
        for _ in range(22):
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

    def point_state(self, v, kappa, ax):
        """Everything worth showing at one point on the track."""
        m = self.m
        ay = v * v * kappa
        fz = self.wheel_loads(v, ax, ay)
        cyf, cyr, cxf, cxr = self._axle_caps(v, ax, ay)
        f_net = m * ax + self.resist(v)           # force the tyres must supply along the car
        d = self.drive
        p_term = 0.0; torque = 0.0; regen = 0.0
        if f_net >= 0:
            if d == "RWD": fxf, fxr = 0.0, f_net
            elif d == "FWD": fxf, fxr = f_net, 0.0
            else:
                tot = cxf + cxr
                fxf, fxr = f_net * cxf / tot, f_net * cxr / tot
            p_term = min(f_net * v / self.eff, self.p_elec)   # clamp step-size rounding
            torque = f_net * self.r / self.gr
        else:
            fb = -f_net
            fxf, fxr = -fb * self.bias, -fb * (1 - self.bias)
            share = (1 - self.bias) if d == "RWD" else (self.bias if d == "FWD" else 1.0)
            if self.regen_w > 0 and v > 1.4 and self.soc < 0.98:      # rules: no regen below 5 km/h
                f_reg = min(fb * share, self.f_torque)
                regen = min(f_reg * v * self.eff, self.regen_w)
                p_term = -regen
                torque = -regen / self.eff / v * self.r / self.gr
        fyf, fyr = m * ay * self.wf, m * ay * (1 - self.wf)
        uf = math.hypot(fyf / cyf, fxf / cxf) if cyf > 0 else 0.0
        ur = math.hypot(fyr / cyr, fxr / cxr) if cyr > 0 else 0.0
        return dict(fz=fz, uf=uf, ur=ur, p=p_term, torque=torque,
                    roll=m * ay * self.h / self.k_roll)
