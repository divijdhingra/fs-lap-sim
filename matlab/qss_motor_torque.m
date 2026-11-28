function T = qss_motor_torque(car, rpm, V_batt)
%QSS_MOTOR_TORQUE  Most torque ONE motor can give (N*m). Thesis eq. 30.
%
%   Above a certain speed an electric motor runs out of voltage to push
%   current in (its "back-EMF" rises with speed) and its torque falls away.
%   This is called flux weakening. So the torque limit depends on motor
%   speed (rpm) and on the battery voltage V_batt.
%
%   With car.torque_map set (fields rpm, volt, T per motor) the thesis
%   2-D lookup table is used (interp2, base MATLAB). Otherwise a simple
%   model: constant torque T_max, then constant power P_motor_max, then zero
%   above top speed, which is rpm_max_nom scaled by V_batt / V_nom.

if ~isempty(car.torque_map)
    mp = car.torque_map;
    T = interp2(mp.rpm, mp.volt, mp.T, rpm, min(max(V_batt, min(mp.volt)), max(mp.volt)), 'linear', 0);
    return
end
omega = max(rpm, 1) * pi / 30;                              % rad/s
T = min(car.T_max, car.P_motor_max / car.n_motors ./ omega);
rpm_top = car.rpm_max_nom * V_batt / car.V_nom;
T(rpm >= rpm_top) = 0;
end
