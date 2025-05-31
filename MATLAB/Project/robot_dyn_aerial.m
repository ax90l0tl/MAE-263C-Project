
%%
function [dXdt,u,F] = robot_dyn(t,X,p)
% sim_t = t; % Uncomment to Output Simulation Time during Runtime

params = p.params;

q = X(1:3);
dq = X(4:6);

M = fcn_Me(q,params);
C = fcn_Ce(q,dq,params);
G = fcn_Ge(q,params);
B = fcn_Be(q,params);
J = fcn_J4(q,params);
dJ = fcn_dJ4(q,dq,params);
p4 = fcn_p4(q,params);

% % Friction Modeling
% tau_Coulomb = p.friction(1:3,1);
% b_viscous = p.friction(4:6,1);
% friction = - diag(tau_Coulomb)*tanh(20*dq) - diag(b_viscous)*dq;
friction = [0 0 0]';

% External Force Applications
% tau = fcn_controller(t,X,p);
tau = [0 0 0]';
% contact = fcn_contact_check;
% f_contact = fcn_contact(t,X,p);
% f_disturb   = fcn_disturbance(t);
f_contact = [0 0]';
f_disturb = [0 0]';
f_tot = f_contact + f_disturb;


% ddq = M \ (B * tau + J' * f_tot + friction - C*dq - G );

if p4(2)<=0
    dq(1) = 0';
    A = [ M -J(2,:)' ; J(2,:) 0];
    
    T = [(B * tau + J' * f_tot + friction - C*dq - G );
        dJ(2,:)*dq];
    sol = A\T;
    ddq = sol(1:3); %+ J'*f_tot ); 
    Fe = sol(4);
else
    ddq = M \ (B * tau + J' * f_tot + friction - C*dq - G );
    Fe = 0;
end
% Fe
dXdt = [dq; ddq];

end

