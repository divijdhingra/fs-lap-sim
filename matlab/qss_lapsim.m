function res = qss_lapsim(car, track, opts)
%QSS_LAPSIM  Point-mass quasi-steady-state lap time simulation (Reinero 2021, ch. 4).
%
%   res = qss_lapsim(car, track) runs one lap (or one run of an open track
%   such as acceleration or skidpad) and returns speed, time and energy
%   along the track.
%
%   car   struct from fs_car_typical / fs_car_logged (all SI units)
%   track struct from load_fs_track / track_from_xy
%   opts  optional struct:
%         .flying   true = flying lap (start at the speed the previous lap
%                   ends with), false = standing start. Default: flying for
%                   a closed track unless track.standing_start is set.
%         .verbose  print a one-line summary (default false)
%
%   "Quasi-steady-state" means that at each short piece of track (here
%   0.25 to 2 m) the car is treated as if it were in a steady state: the
%   forces balance for the current speed and acceleration, and nothing
%   that takes time to build up (body roll, pitch, yaw, tyre heating) is
%   modelled. "Point mass" means the car is a single dot with one mass:
%   no separate front and rear axle, no load transfer between tyres.
%
%   The method, as in the thesis (section 4.3), has five phases:
%     1. g-g-V diagram: how much grip there is at each speed  (qss_ggv)
%     2. Profile 1: fastest steady speed at each point         (qss_corner_speed)
%     3. Profile 2: backwards pass, brake in time for every corner
%     4. Profile 3: forwards pass, accelerate as hard as tyres AND motor
%        AND battery allow, with power and battery voltage updated as
%        charge is used. Final speed = lowest of the three profiles.
%     5. Auxiliary data: time, energy, lateral acceleration, and a check
%        that the car never left its grip envelope (thesis eq. 62).
%
%   Additions to the thesis method (switch off to get the pure thesis model):
%     - driven-axle traction limit for 2WD cars (car.drive = 'AWD' to remove)
%     - battery current limit car.I_max (set Inf to remove)
%     - regenerative braking (car.regen_P_max = 0 to remove)
%     - driver grip shares car.drv_* (set 1 for the thesis's ideal driver)
%
%   Outputs (row vectors along the track, one value per point):
%     res.s  distance (m)          res.v  speed (m/s)       res.t  time (s)
%     res.ax, res.ay  accelerations (m/s^2)
%     res.P_elec  battery power (W, negative = regen)  res.P_mech motor shaft power (W)
%     res.T_motor total motor torque (N*m)   res.V_bp, res.I_bp battery voltage, current
%     res.SOC (%)   res.E_elec cumulative battery energy (J)
%     res.v1, res.v2  speed profiles 1 and 2, for plots
%   and scalars: res.lap_time, res.time (event time: mean of the two timed
%   circles for skidpad), res.energy_kWh, res.v_max, res.ay_max_g, ...

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'verbose'), opts.verbose = false; end
if ~isfield(opts, 'flying'), opts.flying = track.closed && ~track.standing_start; end

g = 9.81;
SOC_MIN = 5;                                   % %, pack treated as empty below this

% ---------- track: unroll to a list of points with segment lengths ----------
N = numel(track.k);
if track.closed
    k  = [track.k, track.k(1)];                % point N+1 = start line again
    ds = track.ds(1:N);
else
    k  = track.k;
    ds = track.ds(1:N-1);
end
np = numel(k);
R  = 1 ./ max(abs(k), 1e-9);

% ---------- 1. grip envelope ----------
OCV0  = interp1(car.SOC_tab, car.OCV_tab, car.SOC_start);
rpm_top = car.rpm_max_nom * OCV0 / car.V_nom;
V_top = rpm_top * pi * car.Rw / (car.gear * 30);           % thesis eq. 9
gg = qss_ggv(car, V_top);
m_eq = gg.m_eq;

% ---------- 2. profile 1: steady cornering speed ----------
v1 = qss_corner_speed(gg, R, V_top);

% ---------- 3. profile 2: braking, run backwards ----------
if track.closed
    % Flying lap (thesis: two loops). Run backwards over two laps so the
    % braking zone into the first corner already knows how the lap ends.
    kk = [k(1:N), k];  dd = [ds, ds];  vv = [v1(1:N), v1];
    v2 = brake_pass(gg, kk, dd, vv);
    v2 = v2(1:np);
else
    v2 = brake_pass(gg, k, ds, v1);
end

% ---------- 4. profile 3: acceleration with powertrain and battery ----------
if track.closed && opts.flying
    % First loop: from rest, battery state frozen, only to find the speed
    % the car crosses the line with. Second loop: the real lap.
    pre = drive_pass(car, gg, k, ds, v1, v2, 0, false);
    v_start = min([pre.v(end), v1(1), v2(1)]);
elseif opts.flying || ~track.standing_start
    v_start = min([track.entry_speed, v1(1), v2(1)]);
else
    v_start = 0;
end
r = drive_pass(car, gg, k, ds, v1, v2, v_start, true);

% ---------- 5. auxiliary data ----------
s = [0, cumsum(ds)];
t = [0, cumsum(r.dt)];
ay = r.v.^2 .* k;
vm = r.v(1:end-1);
ok_acc = r.ax <= gg.ax_acc(ay(1:end-1), vm) + 1e-3;
ok_brk = r.ax >= gg.ax_brk(ay(1:end-1), vm) - 1e-3;
n_bad = sum(~(ok_acc & ok_brk));
if n_bad > 0
    warning('qss_lapsim:ggv', '%d of %d segments outside the grip envelope (thesis eq. 62 check).', n_bad, numel(vm));
end

res.track = track.id;
res.car = car.name;
res.s = s;  res.v = r.v;  res.t = t;
res.ax = [r.ax, r.ax(end)];  res.ay = ay;
res.P_elec = [r.P_elec, 0];  res.P_mech = [r.P_mech, 0];  res.T_motor = [r.T, 0];
res.V_bp = [r.V_bp, r.V_bp(end)];  res.I_bp = [r.I_bp, 0];
res.SOC = r.SOC;  res.E_elec = r.E;  res.E_batt = r.Eb;  res.E_mech = r.Em;
res.v1 = v1;  res.v2 = v2;
res.dt = r.dt;
res.lap_time = t(end);
res.time = t(end);
if ~isempty(track.timed)                       % skidpad: mean of the timed circles
    tt = zeros(1, size(track.timed, 1));
    for j = 1:size(track.timed, 1)
        tt(j) = diff(interp1(s, t, track.timed(j, :)));
    end
    res.timed_laps = tt;
    res.time = mean(tt);
end
res.energy_kWh = r.E(end) / 3.6e6;             % at the battery terminals
res.energy_batt_kWh = r.Eb(end) / 3.6e6;       % including heat inside the pack
res.v_max = max(r.v);  res.v_min = min(r.v);
res.ay_max_g = max(abs(ay)) / g;
res.ax_max_g = max(r.ax) / g;  res.ax_min_g = min(r.ax) / g;
res.P_max_kW = max(r.P_elec) / 1e3;
res.SOC_end = r.SOC(end);
res.ggv_violations = n_bad;
res.m_eq = m_eq;  res.V_top = V_top;
res.gg = gg;
if opts.verbose
    fprintf('%-28s %-16s time %7.3f s  energy %.4f kWh  vmax %5.1f km/h  ay max %.2f g\n', ...
        car.name, track.id, res.time, res.energy_kWh, res.v_max * 3.6, res.ay_max_g);
end
end


% =====================================================================
function v = brake_pass(gg, k, ds, v)
%BRAKE_PASS  Thesis 4.3.3. Walk backwards: from the speed at point i, find
%   the highest speed at i-1 from which the car can still brake down to it,
%   given that cornering at i-1 already uses part of the grip (eq. 20-26).
%   The deceleration needed and the grip both depend on the unknown speed at
%   i-1, so it is found by bisection on the check of eq. 25:
%       (V(i-1)^2 - V(i)^2) / (2 ds) <= |ax_brake(ay(i-1), V(i-1))|
n = numel(v);
for i = n:-1:2
    vi = v(i);
    if v(i-1) <= vi
        continue                               % no braking needed into point i
    end
    d = ds(i-1);  kp = abs(k(i-1));
    need = @(ve) (ve.^2 - vi^2) / (2 * d) + gg.ax_brk(ve.^2 * kp, ve);   % <= 0 means possible
    hi = v(i-1);
    if need(hi) <= 0
        continue                               % profile-1 speed is already brakeable
    end
    lo = vi;
    for it = 1:30
        mid = 0.5 * (lo + hi);
        if need(mid) <= 0, lo = mid; else, hi = mid; end
    end
    v(i-1) = lo;                               % eq. 26: min with profile 1 (lo < v(i-1))
end
end


% =====================================================================
function r = drive_pass(car, gg, k, ds, v1, v2, v_start, energy_on)
%DRIVE_PASS  Thesis 4.3.4. Walk forwards, accelerating as hard as allowed by
%   the tyres (g-g-V), the motors (torque map + power control) and, for a
%   2WD car, the driven tyres. Battery voltage and charge are updated each
%   step when energy_on (thesis "second loop").
n = numel(k);
m = car.m;  m_eq = gg.m_eq;
v = zeros(1, n);  v(1) = v_start;
ax = zeros(1, n-1);  dt = zeros(1, n-1);
P_elec = zeros(1, n-1);  P_mech = zeros(1, n-1);  T = zeros(1, n-1);
V_bp = zeros(1, n-1);  I_bp = zeros(1, n-1);
SOC = car.SOC_start * ones(1, n);
E = zeros(1, n);  Eb = zeros(1, n);  Em = zeros(1, n);
E0 = car.E_total * car.SOC_start / 100;
OCV = interp1(car.SOC_tab, car.OCV_tab, car.SOC_start);
Rin = interp1(car.SOC_tab, car.Rin_tab, car.SOC_start);
Vb_prev = OCV;
mot_force_max = car.n_motors * car.T_max * car.gear / car.Rw;
for i = 1:n-1
    if energy_on                                   % eq. 55-56
        OCV = interp1(car.SOC_tab, car.OCV_tab, max(SOC(i), 0));
        Rin = interp1(car.SOC_tab, car.Rin_tab, max(SOC(i), 0));
    end
    vi = v(i);
    ay = vi^2 * abs(k(i));                         % eq. 27
    rpm = vi * car.gear * 30 / (car.Rw * pi);      % eq. 29
    omega = rpm * pi / 30;
    res_f = gg.resist(vi);                         % drag + rolling resistance, N
    % --- what the motors can do (eq. 30-36) ---
    I_use = min(car.I_max, OCV / (2 * Rin));       % past OCV/(2R) more current gives less power
    P_avail = min(car.P_lim, I_use * (OCV - I_use * Rin));   % battery current limit: addition
    if SOC(i) <= 5, P_avail = 0; end
    T_lim = qss_motor_torque(car, rpm, Vb_prev);
    T_pc = qss_power_control(car, rpm, T_lim, P_avail);
    ax_mot = (car.n_motors * T_pc * car.gear * car.eta_t / car.Rw - res_f) / m_eq;
    % --- what the tyres can do (eq. 37) ---
    ax_tyr = gg.ax_acc(ay, vi);
    ax_trc = traction_limit(car, gg, vi, ay);
    a = min([ax_mot, ax_tyr, ax_trc]);             % eq. 38
    vn = sqrt(max(0, vi^2 + 2 * a * ds(i)));       % eq. 39
    v(i+1) = min([v1(i+1), v2(i+1), vn]);          % eq. 40
    ax(i) = (v(i+1)^2 - vi^2) / (2 * ds(i));       % eq. 41
    % --- power actually used (eq. 42-46) ---
    F_tyre = res_f + m_eq * ax(i);                 % force the tyres put on the road, N
    if F_tyre >= 0
        T(i) = F_tyre * car.Rw / (car.gear * car.eta_t);           % all motors together
        P_mech(i) = T(i) * omega;
        P_elec(i) = P_mech(i) / qss_motor_eff(car, rpm, T(i) / car.n_motors);
    elseif car.regen_P_max > 0 && vi > 1.4 && SOC(i) < 98   % regen: addition (no regen below 5 km/h)
        F_reg = min(-F_tyre * car.regen_share, mot_force_max);
        P_back = min(F_reg * vi * car.eta_t * qss_motor_eff(car, rpm, 0.5 * car.T_max), car.regen_P_max);
        P_elec(i) = -P_back;
        T(i) = -F_reg * car.Rw / car.gear;
        P_mech(i) = T(i) * omega;
    end
    [V_bp(i), I_bp(i)] = qss_battery(P_elec(i), OCV, Rin);          % eq. 47-49
    Vb_prev = V_bp(i);
    v_avg = max(0.5 * (vi + v(i+1)), 0.05);
    dt(i) = ds(i) / v_avg;                         % eq. 50-51
    E(i+1)  = E(i) + dt(i) * P_elec(i);            % eq. 52
    Eb(i+1) = Eb(i) + dt(i) * (P_elec(i) + Rin * I_bp(i)^2);   % eq. 53
    Em(i+1) = Em(i) + dt(i) * P_mech(i);           % eq. 61
    if energy_on
        if car.joule_in_energy, used = Eb(i+1); else, used = E(i+1); end
        SOC(i+1) = (E0 - used) / car.E_total * 100;    % eq. 54
    end
end
r = struct('v', v, 'ax', ax, 'dt', dt, 'P_elec', P_elec, 'P_mech', P_mech, 'T', T, ...
           'V_bp', V_bp, 'I_bp', I_bp, 'SOC', SOC, 'E', E, 'Eb', Eb, 'Em', Em);
end


% =====================================================================
function a = traction_limit(car, gg, v, ay)
%TRACTION_LIMIT  Addition to the thesis. On a 2WD car only the driven tyres
%   can push. Their load is their static share plus their downforce share,
%   plus the load that shifts backwards when accelerating (m*ax*h/L), which
%   itself depends on ax, so a few fixed-point steps are taken. Each driven
%   tyre is assumed to use the same share of its cornering grip as the car.
if strcmpi(car.drive, 'AWD')
    a = Inf;
    return
end
g = 9.81;  m = car.m;
qz = 0.5 * car.rho * car.CzA;
if strcmpi(car.drive, 'RWD')
    w = 1 - car.weight_front;  ab = 1 - car.aero_balance;  sgn = 1;
else
    w = car.weight_front;      ab = car.aero_balance;      sgn = -1;
end
fz_static = m * g * w + qz * v^2 * ab;
e = gg.ell(ay, v);
a = gg.ax_acc(ay, v);
for it = 1:6
    fz = max(0, fz_static + sgn * m * a * car.cg_height / car.wheelbase);
    mu = max(0.4 * car.mu_x, car.mu_x - car.LS_x * (fz / 2 - car.Fz_nom));
    a = (car.drv_trac * mu * fz * e - gg.resist(v)) / gg.m_eq;
end
end
