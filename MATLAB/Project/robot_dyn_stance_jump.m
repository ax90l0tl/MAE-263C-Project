function [dXdt,u_out,F_out] = robot_dyn_stance_jump(t,X,p)
% sim_t = t; % Uncomment to Output Simulation Time during Runtime

params = p.params;
% time_contact = p.time_contact(1,end);
% point_contact = p.point_contact;

[m , n] = size(X);
if n == 6 
    X = X'; % Change from Row to Column Vector
end

N = size(X,2);
u_out = zeros(N,2); % Two Force Inputs
F_out = zeros(N,2); % 2-D Force (X/Y)
dXdt = zeros(size(X,1),N);

% If Used in ODE45 Integrator, N = 1 - else N will be same length as input
for i = 1:N
    q = X(1:3,i);
    dq = X(4:6,i);
    

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
    tau = 10 * controller_jump(t(i),X(:,i),p); % Feedforward Force + Feedback Stabilization
    


    % Torque Saturation (INCOMPLETE) 

    % Data Logging of Controller
    u_out(i,:) = [ tau(2:3) ];
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
    F_out(i,:) = reshape(v(4:5),[1,2]);
    dXdt(:,i) = [dq; ddq];
end

end

