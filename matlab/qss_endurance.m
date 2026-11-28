function out = qss_endurance(car, track, n_laps, verbose)
%QSS_ENDURANCE  Run several laps in a row, carrying battery charge from lap to lap.
%
%   out = qss_endurance(car, track, n_laps) runs n_laps of a closed track
%   (default: track.laps, which is 27 for 'fsuk_endurance', about 22 km).
%   Lap 1 is a standing start if the track says so; later laps are flying.
%   As charge falls, open-circuit voltage falls, so motor top speed and the
%   power the battery can give (current limit) fall too. The run stops if
%   the pack drops below 5 % charge. The thesis simulates one flying lap;
%   this loop is an addition. Tyre wear and heating are NOT modelled here.
%
%   out.lap_time, out.energy_kWh, out.SOC_end   one value per lap
%   out.total_time, out.total_energy_kWh, out.finished, out.laps(i) = full result of lap i

if nargin < 3 || isempty(n_laps), n_laps = track.laps; end
if nargin < 4, verbose = false; end
soc = car.SOC_start;
out.lap_time = [];  out.energy_kWh = [];  out.SOC_end = [];
out.finished = true;
for lap = 1:n_laps
    if soc <= 5
        out.finished = false;
        break
    end
    car.SOC_start = soc;
    r = qss_lapsim(car, track, struct('flying', ~(lap == 1 && track.standing_start)));
    soc = r.SOC_end;
    out.lap_time(end+1) = r.lap_time;
    out.energy_kWh(end+1) = r.energy_kWh;
    out.SOC_end(end+1) = soc;
    out.laps(lap) = r;
    if verbose
        fprintf('lap %2d  %6.2f s  %.3f kWh  SOC %5.1f %%\n', lap, r.lap_time, r.energy_kWh, soc);
    end
end
out.total_time = sum(out.lap_time);
out.total_energy_kWh = sum(out.energy_kWh);
if soc <= 5 && numel(out.lap_time) == n_laps
    out.finished = false;    % finished the distance on an empty pack
end
end
