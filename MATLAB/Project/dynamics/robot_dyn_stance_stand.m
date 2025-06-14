function [dXdt,u_out,F_out,test] = robot_dyn_stance_stand(t,X,p,test)
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
    [tau] = controller_stand(t(i),X(:,i),p); % Feedforward Force + Feedback Stabilization

    % Torque Saturation (INCOMPLETE) 

    % Data Logging of Controller
    u_out(i,:) = [ tau(2:3) ];
    % External Forces (s)
    f_app = [ 0 0 ]';
    
    % Constrained Dynamic Equations
    % A[ddq ; GRF] = B
    % Regular Dynamics w/ Applied Force: M*ddq - J'*GRF= (B * tau + J' * f_app + friction - C*dq - G );
    % 2nd Derivative of Non-Slip Holonomic Constraint : J * dqq + 0 * GRF = -dJ * dq
    % Baumgarte Stabilization to Prevent ODE solver from violating
    % Holonomic Constraint: dJhc*dq + Jhc*ddq + 2*alpha*Jhc*dq + beta^2*p_hc(0)
    % Stops it from drifting through the floor with another sort of tunable
    % PD control with parameters alpha and beta
    alpha = p.alpha_standing;
    beta  = p.beta_standing; % Needed to turn it up, but it affects the solver as well

    A_matrix = [ (M + p.M_rotor), -Jhc';
          Jhc, zeros(2,2)];

    B_vector = [ (B * tau + J14' * f_app + friction - C*dq - G ) ; 
                -dJhc * dq - 2 * alpha * Jhc * dq - beta^2 * fcn_phc(q,params)];
    v = A_matrix \ B_vector;
    ddq = v(1:3);
    F_out(i,:) = reshape(v(4:5),[1,2]);
    dXdt(:,i) = [dq; ddq];
end

end

