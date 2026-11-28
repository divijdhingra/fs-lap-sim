function V1 = qss_corner_speed(gg, R, V_max)
%QSS_CORNER_SPEED  Speed profile 1: highest steady speed at each point. Thesis 4.3.2.
%
%   V1 = qss_corner_speed(gg, R, V_max) gives, for every curvature radius R
%   (m), the fastest speed the car could hold if that bend went on forever,
%   ignoring what comes before and after:  V^2 / R = ay(V)  (eq. 18-19).
%
%   Because grip rises with speed (downforce), ay depends on V, so the thesis
%   steps V down from V_max until V^2/R fits under the grip. Here the same
%   test is solved by bisection (halving the interval 40 times), for all
%   points at once. A straight (R = Inf) gets V_max, the motor top speed.

R = R(:)';
lo = zeros(size(R));
hi = V_max * ones(size(R));
fits = @(V) V.^2 ./ R <= gg.ay_steady(V);
ok = fits(hi);                       % already fits at top speed: done
for it = 1:40
    mid = 0.5 * (lo + hi);
    f = fits(mid);
    lo(f) = mid(f);
    hi(~f) = mid(~f);
end
V1 = lo;
V1(ok) = V_max;
end
