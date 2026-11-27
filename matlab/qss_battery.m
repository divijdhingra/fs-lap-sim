function [V_bp, I_bp] = qss_battery(P_elec, OCV, R_in)
%QSS_BATTERY  Battery terminal voltage and current for a power draw. Thesis eq. 47-49.
%
%   The pack is an ideal voltage source (its open-circuit voltage OCV, set
%   by state of charge) in series with an internal resistance R_in. Drawing
%   power P = V*I with V = OCV - R_in*I gives a quadratic in I:
%       I = (OCV - sqrt(OCV^2 - 4*R_in*P)) / (2*R_in)          (eq. 48)
%       V = OCV - R_in*I                                        (eq. 49)
%   Negative P (regen) gives negative I (charging) and V above OCV.
%   If P is more than the pack can give (OCV^2/(4 R_in)) the root is clipped.

disc = max(0, OCV.^2 - 4 * R_in .* P_elec);
I_bp = (OCV - sqrt(disc)) ./ (2 * R_in);
V_bp = OCV - R_in .* I_bp;
end
