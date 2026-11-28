function car = fs_car_logged()
%FS_CAR_LOGGED  Typical car with the figures fitted to the logged FS UK 2025 stint.
%
%   Same values as the website preset "Fitted to the logged FS UK stint".
%   Read from the log: 46 kW peak power, no regen, about 2000 N push at full
%   throttle (torque limited). Fitted on the website: grip 1.32, driver using
%   79 % of it cornering and on exit, 48 % braking, 74 % drivetrain
%   efficiency. Mass (300 kg) and the rest are still ASSUMED.

car = fs_car_typical();
car.name = 'Fitted to logged FS UK stint';
car.T_max     = 80;              % N*m
car.eta_motor = 0.74 / car.eta_t; % so eta_t*eta_motor = 0.74
car.regen_P_max = 0;
car.mu_x = 1.32;  car.mu_y = 1.32;
car.LS_x = car.mu_x * 0.15 / car.Fz_nom;
car.LS_y = car.mu_y * 0.15 / car.Fz_nom;
car.P_lim = 46e3;
car.drv_lat = 0.79;  car.drv_brake = 0.48;  car.drv_trac = 0.79;
end
