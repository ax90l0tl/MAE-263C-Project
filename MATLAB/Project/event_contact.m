function [zeroCrossing,isterminal,direction] = fcn_contact(t,X,p)

params = p.params;
q = X(1:3);
p4 = fcn_p4(q,params);

% Assumptions of Flat Ground
zeroCrossing = p4(2); % Y-coordinate
isterminal = 1; % Results in termination when condition met
direction = 1; % Only from Positive Axis