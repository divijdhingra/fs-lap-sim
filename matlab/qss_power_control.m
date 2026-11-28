function T = qss_power_control(car, rpm, T_lim, P_avail)
%QSS_POWER_CONTROL  Trim motor torque so electrical power stays under the limit. Thesis eq. 31-35.
%
%   The car's controller caps power drawn from the battery (FS rules: 80 kW).
%   Electrical power for torque T per motor at speed rpm is
%       P_elec = n_motors * T * omega / eta_motor(rpm, T)     (eq. 33-34)
%   If that is over P_avail, torque is reduced until it fits (eq. 35).
%   The thesis reduces it in fixed small steps; here bisection does the
%   same search to a tighter tolerance.

omega = rpm * pi / 30;
pe = @(t) car.n_motors * t * omega / qss_motor_eff(car, rpm, t);
T = T_lim;
if T <= 0 || pe(T) <= P_avail
    return
end
lo = 0; hi = T_lim;
for it = 1:30
    mid = 0.5 * (lo + hi);
    if pe(mid) <= P_avail, lo = mid; else, hi = mid; end
end
T = lo;
end
