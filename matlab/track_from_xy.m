function track = track_from_xy(x, y, closed, name)
%TRACK_FROM_XY  Build a track struct from centre-line coordinates (thesis 4.2.1).
%
%   track = track_from_xy(x, y, closed) takes points in metres, in driving
%   order, roughly evenly spaced (0.5 to 5 m, as the thesis suggests).
%   Curvature radius R(i) is the radius of the circle through points i-1,
%   i and i+1. ds(i) is the distance from point i to point i+1.
%   Noisy GPS data should be smoothed first, or R will be noisy too.

if nargin < 3, closed = true; end
if nargin < 4, name = 'custom'; end
x = x(:)';  y = y(:)';
n = numel(x);
ip = [n 1:n-1];  in = [2:n 1];                 % previous and next point (wrapping round)
% signed curvature of the circle through three points: 2*cross / (|ab||bc||ca|)
ax = x(ip); ay = y(ip); cx = x(in); cy = y(in);
cr = (x - ax) .* (cy - ay) - (y - ay) .* (cx - ax);
d1 = hypot(x - ax, y - ay); d2 = hypot(cx - x, cy - y); d3 = hypot(cx - ax, cy - ay);
k = 2 * cr ./ max(d1 .* d2 .* d3, eps);
if ~closed                                      % no neighbour past the ends: copy the next one in
    k(1) = k(2);  k(n) = k(n-1);
end

track.id = name; track.name = name;
track.x = x; track.y = y; track.k = k; track.R = 1 ./ abs(k);
if closed
    track.ds = hypot(x([2:n 1]) - x, y([2:n 1]) - y);
else
    track.ds = [hypot(diff(x), diff(y)) 0];
end
track.closed = closed;
track.standing_start = ~closed;
track.entry_speed = 10;
track.timed = [];
track.laps = 1;
end
