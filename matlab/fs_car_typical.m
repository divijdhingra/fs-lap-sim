function car = fs_car_typical()
%FS_CAR_TYPICAL  Point-mass parameters for a typical electric FS car (SI units).
%
%   car = fs_car_typical() returns a struct with every number the point-mass
%   quasi-steady-state (QSS) solver needs. The figures are the website's
%   "Typical FS electric car" defaults (fslap/params.py), turned into the
%   form used in the thesis (Reinero 2021, section 4.2.2), so the MATLAB and
%   web results can be compared like for like.
%
%   Edit any field, or copy this file to make your own car.
%   Fields marked ASSUMED are not on the website and are my estimates.

car.name = 'Typical FS electric car';

% ---------------- Mass ----------------
car.m          = 300;    % kg, car 230 + driver 70
car.J_wheel    = 0.08;   % kg*m^2, spin inertia of ONE wheel + tyre + hub (ASSUMED)
car.n_wheels   = 4;
car.J_motor    = 0.02;   % kg*m^2, spin inertia of ONE motor rotor (ASSUMED)
                         % Set both inertias to 0 to match the website, which ignores them.

% ------------- Layout (only used for the driven-axle traction limit) -------------
car.drive        = 'RWD';  % 'RWD', 'FWD' or 'AWD'. 'AWD' = pure thesis point mass.
car.wheelbase    = 1.55;   % m
car.cg_height    = 0.30;   % m
car.weight_front = 0.47;   % -, static share of weight on the front axle
car.aero_balance = 0.45;   % -, share of downforce on the front axle

% ---------------- Tyres ----------------
% Thesis model: peak friction coefficient at a nominal tyre load, falling
% linearly as load rises (load sensitivity, LS):
%     mu = mu_nominal - LS * (Fz_tyre - Fz_nom)
car.mu_x   = 1.45;       % -, peak longitudinal friction (accelerating/braking)
car.mu_y   = 1.50;       % -, peak lateral friction (cornering)
car.Fz_nom = 750;        % N, tyre load at which mu_x and mu_y apply
% Website "load sensitivity 0.15" = 15 % grip lost when the load doubles.
car.LS_x   = car.mu_x * 0.15 / car.Fz_nom;   % 1/N, = 2.9e-4
car.LS_y   = car.mu_y * 0.15 / car.Fz_nom;   % 1/N, = 3.0e-4
car.f_rr   = 0.02;       % -, rolling resistance force / vertical load

% ---------------- Aerodynamics ----------------
car.rho = 1.225 * 288.15 / (273.15 + 20);   % kg/m^3, air at 20 C (= 1.204)
car.CzA = 3.2;           % m^2, downforce coefficient x area (website ClA)
car.CxA = 1.35;          % m^2, drag coefficient x area (website CdA)

% ---------------- Driver ----------------
% Share of the tyres' grip the driver uses. 1 = the thesis's ideal driver.
car.drv_lat   = 1.0;     % cornering
car.drv_brake = 1.0;     % braking
car.drv_trac  = 1.0;     % putting power down

% ---------------- Powertrain ----------------
car.n_motors    = 1;         % number of motors (thesis car: 4)
car.T_max       = 230;       % N*m, peak torque of ONE motor
car.P_motor_max = 100e3;     % W, peak mechanical power, all motors together
car.rpm_max_nom = 6000;      % rpm, motor top speed at nominal pack voltage
car.gear        = 3.8;       % -, motor turns per wheel turn (thesis tau)
car.Rw          = 0.203;     % m, loaded tyre radius
car.eta_t       = 0.97;      % -, gearbox efficiency (ASSUMED split of the website's 88 %)
car.eta_motor   = 0.88 / 0.97; % -, motor + inverter efficiency, so eta_t*eta_motor = 0.88
% Optional lookup tables (thesis eq. 30 and 32). Leave empty to use the
% simple model: torque = min(T_max, P_motor_max/omega), cut to zero above
% top speed, which scales with battery voltage; constant efficiency.
car.torque_map = [];         % struct with fields rpm (1xA), volt (1xB), T (BxA) per motor
car.eff_map    = [];         % struct with fields rpm (1xA), T (1xB), eta (BxA)

% ---------------- Power limits ----------------
car.P_lim = 80e3;            % W, electrical power limit at the battery (FS rules: 80 kW)
car.I_max = 170;             % A, battery discharge current limit

% ---------------- Battery (thesis section 3.0.2 and eq. 47-57) ----------------
n_series      = 140;                            % cells in series
car.V_nom     = n_series * 3.7;                 % V, nominal pack voltage (518 V)
car.E_total   = 6.5 * 3.6e6;                    % J, usable capacity (6.5 kWh)
car.SOC_start = 100;                            % %, state of charge at the start
car.SOC_tab   = [0 5 10 20 30 50 70 80 90 100]; % %
car.OCV_tab   = n_series * [3.00 3.30 3.45 3.55 3.60 3.70 3.85 3.95 4.05 4.20]; % V, open-circuit voltage
car.Rin_tab   = 0.20 * ones(size(car.SOC_tab)); % ohm, pack internal resistance
car.joule_in_energy = true;  % thesis eq. 53: count R*I^2 heat in the battery energy used

% ---------------- Regenerative braking (not in the thesis point-mass) ----------------
car.regen_P_max = 20e3;      % W, most electrical power put back while braking (0 = off)
car.regen_share = 0.35;      % -, share of braking force on the driven wheels (RWD with 65 % front bias)
end
