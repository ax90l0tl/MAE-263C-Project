function F = fcn_F_traj(s,delay,duration,F_max)
% 6th Order Polynomial Function with Zero Initial and Final P,V,A
% At s = 0.5, the function reaches F_max
s = (s-delay)/duration;
F = F_max * s.^3 .* (-64*s.^3 + 192*s.^2 - 192*s + 64);
if F<0
    F = 0;
end
