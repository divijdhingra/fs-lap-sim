function eta = qss_motor_eff(car, rpm, T)
%QSS_MOTOR_EFF  Motor + inverter efficiency at a speed and torque. Thesis eq. 32.
%
%   eta = electrical power in / mechanical power out of the motor shaft.
%   With car.eff_map (fields rpm, T, eta) the thesis lookup table is used;
%   otherwise the constant car.eta_motor.

if ~isempty(car.eff_map)
    mp = car.eff_map;
    eta = interp2(mp.rpm, mp.T, mp.eta, min(max(rpm, min(mp.rpm)), max(mp.rpm)), ...
                  min(max(abs(T), min(mp.T)), max(mp.T)), 'linear');
    eta = max(eta, 0.05);
else
    eta = car.eta_motor * ones(size(rpm));
end
end
