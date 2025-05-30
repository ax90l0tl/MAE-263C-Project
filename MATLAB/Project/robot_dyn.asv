
%%
function dXdt = robot_dyn(t,X,p)
sim_t = t;

params = p.params;

q = X(1:3);
dq = X(4:6);

M = fcn_Me(q,params);
C = fcn_Ce(q,dq,params);
G = fcn_Ge(q,params);
B = fcn_Be(q,params);
J = fcn_J4(q,params);
dJ = fcn_dJ4(q,dq,params);

% % Friction Modeling
% tau_Coulomb = p.friction(1:3,1);
% b_viscous = p.friction(4:6,1);
% friction = - diag(tau_Coulomb)*tanh(20*dq) - diag(b_viscous)*dq;
friction = [0 0 0]';

% External Force Applications
% tau = fcn_controller(t,X,p);
tau = [0 0 0]';
%f_contact = fcn_contact(t,X,p);
%f_disturb   = fcn_disturbance(t,X,p);
%f_tot = f_contact + f_disturb

A = [ M -J' ; J zeros(2,2)]

ddq = M \ (B * tau + friction - C*dq - G ); %+ J'*f_tot );

dXdt = [dq; ddq];
end