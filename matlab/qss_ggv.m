function gg = qss_ggv(car, V_max, N_vel, N_ell)
%QSS_GGV  Grip envelope (g-g-V diagram) of the point-mass car. Thesis section 4.3.1.
%
%   gg = qss_ggv(car, V_max) returns function handles that answer "how
%   hard can the tyres accelerate, brake or corner at this speed?", plus a
%   table of points for plotting.
%
%   A g-g-V diagram is the set of longitudinal (ax) and lateral (ay)
%   accelerations the car can reach at each speed V. It grows with speed
%   because downforce pushes the tyres into the road.
%
%   Physics (thesis eq. 10-13):
%     Fz = m*g + 0.5*rho*CzA*V^2              total vertical load
%     mu = mu_nom - LS*(Fz/4 - Fz_nom)         load sensitivity, per tyre
%     friction ellipse:  (Fx/(mu_x*Fz))^2 + (Fy/(mu_y*Fz))^2 <= 1
%     ax = (Fx - drag - rolling resistance) / m_eq
%     ay = Fy / m
%   m_eq (thesis eq. 3) adds the spin inertia of wheels and motors, which
%   resists speeding up and slowing down but not cornering.
%
%   The thesis samples the ellipse in N_ell points and interpolates with
%   scatteredInterpolant. The ellipse has an exact formula, so here the
%   handles evaluate it directly: same answer, no interpolation error, and
%   no Curve Fitting / Statistics toolbox needed. The table (gg.table) is
%   still built the thesis way, for plotting.
%
%   Handles (all accept arrays, V in m/s, accelerations in m/s^2):
%     gg.Fz(V), gg.mux(V), gg.muy(V)
%     gg.ay_pure(V)       most cornering with no braking or drive force at all
%     gg.ay_steady(V)     most cornering while holding speed (tyres also push
%                         against drag) = the GGV at ax = 0 (thesis eq. 14)
%     gg.ax_acc(ay, V)    most acceleration the tyres allow while cornering at ay
%     gg.ax_brk(ay, V)    most braking (negative number) while cornering at ay

if nargin < 3, N_vel = 30; end
if nargin < 4, N_ell = 40; end
g = 9.81;
m = car.m;
m_eq = car.m + car.n_wheels * car.J_wheel / car.Rw^2 ...
             + car.n_motors * car.J_motor * car.gear^2 / car.Rw^2;    % thesis eq. 3
qz = 0.5 * car.rho * car.CzA;
qx = 0.5 * car.rho * car.CxA;

Fz  = @(V) m * g + qz * V.^2;                                          % eq. 10
mu_floor = 0.4;                                                        % never below 40 % of nominal
mux = @(V) max(mu_floor * car.mu_x, car.mu_x - car.LS_x * (Fz(V) / 4 - car.Fz_nom));
muy = @(V) max(mu_floor * car.mu_y, car.mu_y - car.LS_y * (Fz(V) / 4 - car.Fz_nom));
resist = @(V) qx * V.^2 + car.f_rr * Fz(V);                            % drag + rolling resistance, N

% The driver uses a share of the grip: a smaller ellipse.
ay_pure = @(V) car.drv_lat * muy(V) .* Fz(V) / m;
fx_acc  = @(V) car.drv_trac  * mux(V) .* Fz(V);                       % N, pure drive grip
fx_brk  = @(V) car.drv_brake * mux(V) .* Fz(V);                       % N, pure brake grip
ell     = @(ay, V) sqrt(max(0, 1 - (abs(ay) ./ ay_pure(V)).^2));       % eq. 11, share of Fx left

gg.m_eq = m_eq;
gg.Fz = Fz; gg.mux = mux; gg.muy = muy; gg.resist = resist;
gg.ay_pure = ay_pure;
gg.ell = ell;
gg.ax_acc = @(ay, V) (fx_acc(V) .* ell(ay, V) - resist(V)) / m_eq;    % eq. 12, positive side
gg.ax_brk = @(ay, V) (-fx_brk(V) .* ell(ay, V) - resist(V)) / m_eq;   % eq. 12, negative side
% Steady cornering: the tyres must also push with Fx = resist to hold speed,
% so ay is the ellipse at Fx = resist (the GGV surface at ax = 0).
gg.ay_steady = @(V) ay_pure(V) .* sqrt(max(0, 1 - (resist(V) ./ fx_acc(V)).^2));

% Thesis-style table: N_vel speeds x N_ell points x (accel, brake) -> [V ax ay]
Vv = linspace(0, V_max, N_vel);
tab = zeros(N_vel * N_ell * 2, 3);
r = 0;
for a = 1:N_vel
    ays = linspace(0, ay_pure(Vv(a)), N_ell);
    for sgn = [1 -1]
        if sgn > 0, axs = gg.ax_acc(ays, Vv(a)); else, axs = gg.ax_brk(ays, Vv(a)); end
        tab(r + (1:N_ell), :) = [Vv(a) * ones(N_ell, 1), axs(:), ays(:)];
        r = r + N_ell;
    end
end
gg.table = tab;
end
