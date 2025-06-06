function F = fcn_F_traj(s,F_max)
% 6th Order Polynomial Function with Zero Initial and Final P,V,A
% At s = 0.5, the function reaches F_max
F = F_max * s^3 .* (-64*s^3 + 192*s^2 - 192*s + 64);
