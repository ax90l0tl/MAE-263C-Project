function [value, isterminal, direction] = floor_contact(t,X,p)

% p = get_params();
params = p.params;

q = X(1:3);            %joint angles vector
dq = X(4:6);           %joint velocities vector
p = fcn_p4(q,params);  %End-effector spatial position

%Guard function: contact occurs when: value = 0
value      = p(2) - 0.35; %MODIFY THIS FOR ITEM 2  
isterminal = 1;	%Stop the integration
direction  = 1;
end