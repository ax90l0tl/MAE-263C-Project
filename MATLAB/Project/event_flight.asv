function [zeroCrossing,isterminal,direction] = event_flight(t,X,p)

params = p.params;

q = X(1:3,:);
dq = X(4:6,:);

% Leg Dynamics
M = fcn_Me(q,params);
C = fcn_Ce(q,dq,params);
G = fcn_Ge(q,params);
B = fcn_Be(q,params);

% Foot to Hip Jacobian
J14 = fcn_J14(q,params);

% Contact Constraint Jacobians
Jhc = fcn_Jhc(q,params);
dJhc = fcn_dJhc(q,dq,params);

% Friction Modeling (INCOMPLETE)
% tau_Coulomb = p.friction(1:3,1);
% b_viscous = p.friction(4:6,1);
% friction = - diag(tau_Coulomb)*tanh(20*dq) - diag(b_viscous)*dq;
friction = [0 0 0]';

% Controller Torque
tau = controller_jump(t,X,p); % Feedforward Force + Feedback Stabilization

% Torque Saturation (INCOMPLETE) 

% External Forces (s)
f_app = [ 0 0 ]';

% Constrained Dynamic Equations
% A[ddq ; GRF] = B
% Regular Dynamics w/ Applied Force: M*ddq - J'*GRF= (B * tau + J' * f_app + friction - C*dq - G );
% 2nd Derivative of Non-Slip Holonomic Constraint : J * dqq + 0 * GRF = -dJ * dq
A_matrix = [ M , -Jhc';
      Jhc, zeros(2,2)];
B_vector = [ (B * tau + J14' * f_app + friction - C*dq - G ) ; % Re-Evaluate Jacobian in Front of f_app to see if the right type
            -dJhc * dq];
v = A_matrix \ B_vector;
ddq = v(1:3);
GRF = reshape(v(4:5),[1,2]);

GRFy = GRF(2);
threshold = p.threshold; % 2.5% of Static Weight 


% Assumptions of Flat Ground
zeroCrossing = GRFy - threshold ; % Y-Force > Some Threshhold (<< Static Weight)
isterminal = 1; % Results in termination when condition met
direction = -1; % Can Occur when from Positive or Negative Crossing

end