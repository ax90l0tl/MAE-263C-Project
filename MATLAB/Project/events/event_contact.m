function [zeroCrossing,isterminal,direction] = event_contact(t,X,p)

params = p.params;
q = X(1:3);
p4 = fcn_p4(q,params);
% % Ensure Simulation Time does not affect
% if p4(2)<=0
%     p4(2)=0;
% end
% Assumptions of Flat Ground
zeroCrossing = p4(2); % Y-coordinate
isterminal = 1; % Results in termination when condition met
direction = -1; % Can Occur when from Positive or Negative Crossing