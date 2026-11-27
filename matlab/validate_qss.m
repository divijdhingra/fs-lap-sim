%VALIDATE_QSS  Checks the MATLAB point-mass sim against hand calculations,
%   Formula Student benchmarks, the website engine and the logged FS UK stint.
%   Prints PASS / FAIL for each check. Takes about 10 s in MATLAB.

here = fileparts(mfilename('fullpath'));
g = 9.81;
report = @(name, val, lo, hi, unit) fprintf('%-58s %9.3f %-5s [%g, %g]  %s\n', ...
    name, val, unit, lo, hi, pass_fail(val >= lo && val <= hi));

fprintf('\n--- 1. Hand calculations (textbook answers the solver must reproduce) ---\n');
% Skidpad, no wings, no load sensitivity, almost no drag: v = sqrt(mu*g*R).
c = fs_car_typical();
c.CzA = 0; c.CxA = 0.01; c.f_rr = 0.001; c.LS_x = 0; c.LS_y = 0; c.mu_y = 1.4;
c.J_wheel = 0; c.J_motor = 0; c.drive = 'AWD';
r = qss_lapsim(c, load_fs_track('skidpad'));
Rsk = 9.125;                                   % m, skidpad driving line radius
t_hand = 2 * pi * Rsk / sqrt(1.4 * g * Rsk);
report('Skidpad time / hand calc 2*pi*R/sqrt(mu*g*R)', r.time / t_hand, 0.99, 1.01, '-');
% Acceleration, grip limited only: t = sqrt(2*L/(mu*g)).
c = fs_car_typical();
c.CzA = 0; c.CxA = 0; c.f_rr = 0; c.LS_x = 0; c.mu_x = 1.4; c.drive = 'AWD';
c.J_wheel = 0; c.J_motor = 0; c.T_max = 5000; c.rpm_max_nom = 30000; c.P_motor_max = 1e7; c.P_lim = 1e7; c.I_max = 1e5;
tr = load_fs_track('accel');
r = qss_lapsim(c, tr);
t_hand = sqrt(2 * r.s(end) / (1.4 * g));
report('Acceleration time / hand calc sqrt(2L/(mu*g))', r.time / t_hand, 0.99, 1.01, '-');

fprintf('\n--- 2. Formula Student benchmarks (typical electric car) ---\n');
car = fs_car_typical();
acc = qss_lapsim(car, load_fs_track('accel'));
skd = qss_lapsim(car, load_fs_track('skidpad'));
aut = qss_lapsim(car, load_fs_track('fsuk_autocross'));
% Competitive electric RWD cars: 75 m in about 3.6-4.6 s; winged skidpad about 4.7-5.4 s.
report('Acceleration 75 m (s)', acc.time, 3.6, 4.6, 's');
report('Skidpad (s)', skd.time, 4.7, 5.4, 's');
report('Power never above 80 kW (peak, kW)', max([acc.P_max_kW skd.P_max_kW aut.P_max_kW]), 0, 80.05, 'kW');
report('Segments outside grip envelope (thesis eq. 62)', acc.ggv_violations + skd.ggv_violations + aut.ggv_violations, 0, 0, '');

fprintf('\n--- 3. Against the website engine (fslap, same car figures) ---\n');
% Website results for the same car, run 27 Nov 2026. The website models four
% tyre loads, load transfer, axle balance, tyre slip and temperature, so a
% point mass should be FASTER (thesis: 5-7 %% faster than real car / transient sim).
web = struct('accel', 4.117, 'skidpad', 5.142, 'fsuk_autocross', 52.207, 'autox_kWh', 0.3915);
fprintf('%-20s %10s %10s %8s\n', 'event', 'MATLAB', 'website', 'diff');
fprintf('%-20s %10.3f %10.3f %+7.1f%%\n', 'accel (s)', acc.time, web.accel, 100 * (acc.time / web.accel - 1));
fprintf('%-20s %10.3f %10.3f %+7.1f%%\n', 'skidpad (s)', skd.time, web.skidpad, 100 * (skd.time / web.skidpad - 1));
fprintf('%-20s %10.3f %10.3f %+7.1f%%\n', 'autocross (s)', aut.time, web.fsuk_autocross, 100 * (aut.time / web.fsuk_autocross - 1));
fprintf('%-20s %10.4f %10.4f %+7.1f%%\n', 'autocross (kWh)', aut.energy_kWh, web.autox_kWh, 100 * (aut.energy_kWh / web.autox_kWh - 1));

fprintf('\n--- 4. Against the logged FS UK 2025 endurance stint ---\n');
logd = jsondecode(fileread(fullfile(here, '..', 'data', 'validation.json')));
trk = load_fs_track('fsuk_endurance');
lg = qss_lapsim(fs_car_logged(), trk, struct('flying', true));
id = qss_lapsim(fs_car_typical(), trk, struct('flying', true));
e_log = logd.lap_energy_kwh(logd.best_lap);
n = numel(logd.v);
vs = 0.5 * (lg.v(1:n) + lg.v(2:n+1));         % segment mean speed, as the website logs it
rms_kmh = sqrt(mean((vs(:) - logd.v(:)).^2)) * 3.6;
cc = corrcoef(vs(:), logd.v(:));
fprintf('%-34s %10s %10s %8s\n', '', 'MATLAB', 'logged', 'diff');
fprintf('%-34s %10.2f %10.2f %+7.1f%%\n', 'Lap time, fitted car (s)', lg.lap_time, logd.best_time, 100 * (lg.lap_time / logd.best_time - 1));
fprintf('%-34s %10.4f %10.4f %+7.1f%%\n', 'Energy per lap, fitted car (kWh)', lg.energy_kWh, e_log, 100 * (lg.energy_kWh / e_log - 1));
fprintf('%-34s %10.2f %10.2f %+7.1f%%\n', 'Lap time, ideal typical car (s)', id.lap_time, logd.best_time, 100 * (id.lap_time / logd.best_time - 1));
fprintf('%-34s %10.2f %10s\n', 'Speed trace RMS error (km/h)', rms_kmh, '');
fprintf('%-34s %10.3f %10s\n', 'Speed trace correlation', cc(1, 2), '');
fprintf('%-34s %10.2f %10.2f\n', 'Peak lateral (g)', lg.ay_max_g, logd.peak_lat_g);
fprintf('%-34s %10.2f %10.2f\n', 'Peak braking (g)', -lg.ax_min_g, logd.peak_brake_g);
fprintf('%-34s %10.1f %10.1f\n', 'Peak power (kW)', lg.P_max_kW, logd.peak_power_kw);
fprintf('(Lap time and energy depend on figures that were FITTED to this log on the website;\n');
fprintf(' the shape of the speed trace was not fitted.)\n\n');

if exist('OCTAVE_VERSION', 'builtin') == 0 && usejava('desktop')
    plot_lap(lg, trk, logd.v);
end
