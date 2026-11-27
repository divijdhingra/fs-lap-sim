%RUN_LAP  Start here. Runs the point-mass QSS sim on one track and plots it.
%
%   Open this file in MATLAB and press Run (or type run_lap in the Command
%   Window with this folder as the current folder). Change the three
%   lines below to try another car or track.
%   load_fs_track() with no input lists the track ids.

car   = fs_car_typical();       % or fs_car_logged(), or your own copy
track = load_fs_track('fsuk_autocross');
opts  = struct('verbose', true);

% Example edits (uncomment):
% car.m   = 280;                % kg
% car.CzA = 4.0;                % m^2, more downforce
% car.drive = 'AWD';            % pure thesis point mass, all four wheels driven

res = qss_lapsim(car, track, opts);
plot_lap(res, track);

% Endurance (27 laps of FS UK, about 22 km) -- takes a little longer:
% e = qss_endurance(car, load_fs_track('fsuk_endurance'), [], true);
