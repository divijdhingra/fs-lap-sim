function fig = plot_lap(res, track, log_v)
%PLOT_LAP  Four-panel summary of a qss_lapsim result.
%
%   plot_lap(res, track) draws:
%     1. speed against distance, with the three thesis speed profiles
%        (corner limit, braking, final) like thesis figure 26
%     2. track map coloured by speed
%     3. the g-g diagram: lateral vs longitudinal acceleration at every
%        point, over the grip envelope at the lap's average speed
%     4. battery power and state of charge against distance
%   plot_lap(res, track, log_v) also draws a logged speed trace (m/s, one
%   value per track point) on panel 1.

if nargin < 3, log_v = []; end
g = 9.81;
fig = figure('Color', 'w', 'Position', [100 100 1300 820]);
try
    theme(fig, 'light');            % MATLAB R2025a+: ignore dark mode for this figure
catch
end
kmh = 3.6;

subplot(2, 2, 1);
cap = 1.15 * max(res.v) * kmh;
plot(res.s, min(res.v1 * kmh, cap), ':', 'Color', [0.6 0.6 0.6]); hold on
plot(res.s, min(res.v2 * kmh, cap), '--', 'Color', [0.85 0.45 0.1]);
plot(res.s, res.v * kmh, '-', 'Color', [0.1 0.35 0.8], 'LineWidth', 1.5);
leg = {'1: corner limit', '2: braking', '3: final (sim)'};
if ~isempty(log_v)
    n = min(numel(log_v), numel(res.s));
    plot(res.s(1:n), log_v(1:n) * kmh, '-', 'Color', [0.2 0.2 0.2]);
    leg{end+1} = 'logged';
end
ylim([0 cap]); xlim([0 res.s(end)]);
xlabel('Distance (m)'); ylabel('Speed (km/h)'); grid on
legend(leg, 'Location', 'southoutside', 'Orientation', 'horizontal');
title(sprintf('%s: %.3f s, %.3f kWh', strrep(res.track, '_', ' '), res.time, res.energy_kWh));
light_axes(gca);

subplot(2, 2, 2);
n = numel(track.x);
scatter(track.x, track.y, 12, res.v(1:n) * kmh, 'filled');
axis equal; grid on; colorbar;
xlabel('x (m)'); ylabel('y (m)'); title('Speed on track (km/h)');
light_axes(gca);

subplot(2, 2, 3);
vbar = mean(res.v);
ays = linspace(0, res.gg.ay_pure(vbar), 80);
env_a = res.gg.ax_acc(ays, vbar) / g;  env_b = res.gg.ax_brk(ays, vbar) / g;
plot([-fliplr(ays) ays] / g, [fliplr(env_a) env_a], 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); hold on
plot([-fliplr(ays) ays] / g, [fliplr(env_b) env_b], 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
plot(res.ay / g, res.ax / g, '.', 'Color', [0.1 0.35 0.8], 'MarkerSize', 6);
axis equal; grid on
xlabel('Lateral acceleration (g)'); ylabel('Longitudinal acceleration (g)');
title(sprintf('g-g diagram (grey: tyre grip at %.0f km/h)', vbar * kmh));
light_axes(gca);

subplot(2, 2, 4);
plot(res.s, res.P_elec / 1e3, 'Color', [0.8 0.2 0.2], 'LineWidth', 0.8); hold on
ylabel('Battery power (kW)'); xlabel('Distance (m)'); grid on; xlim([0 res.s(end)]);
yyaxis right
plot(res.s, res.SOC, 'Color', [0.1 0.55 0.3], 'LineWidth', 1.5);
ylabel('State of charge (%)');
set(gca, 'YColor', [0.1 0.55 0.3]);
yyaxis left
set(gca, 'YColor', [0.8 0.2 0.2]);
title(sprintf('Power and charge (peak %.1f kW)', res.P_max_kW));
light_axes(gca, true);
end


function light_axes(ax, keep_ycolor)
%LIGHT_AXES  White background and dark text, so the plots read well in
%   MATLAB's dark mode as well as light mode.
if nargin < 2, keep_ycolor = false; end
ink = [0.15 0.15 0.15];
set(ax, 'Color', 'w', 'XColor', ink, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6, 'FontSize', 10);
if ~keep_ycolor, set(ax, 'YColor', ink); end
set(get(ax, 'Title'), 'Color', ink, 'FontWeight', 'bold');
set(get(ax, 'XLabel'), 'Color', ink);
if ~keep_ycolor, set(get(ax, 'YLabel'), 'Color', ink); end
lg = findobj(get(ax, 'Parent'), 'Type', 'legend');
for j = 1:numel(lg)
    set(lg(j), 'Color', 'w', 'TextColor', ink, 'EdgeColor', [0.8 0.8 0.8]);
end
cb = findobj(get(ax, 'Parent'), 'Type', 'colorbar');
for j = 1:numel(cb)
    set(cb(j), 'Color', ink);
end
end
