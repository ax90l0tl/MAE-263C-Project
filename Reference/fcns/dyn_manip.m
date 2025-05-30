function dXdt = dyn_manip(t,X,p)
Simulation_time = t %current simulation time

%Robot parameters
params = p.params;

%Current state
q = X(1:3);     %joint angles vector
dq = X(4:6);    %joint velocities vector

%Calculating the matrices for the equation of motion:
De = fcn_De(q,params);      %Inertia metrix
Ce = fcn_Ce(q,dq,params);   %Coriolis matrix
Ge = fcn_Ge(q,params);      %Gravity vector
Be = fcn_Be(q,params);      %Torque selection (identity)

%Joint friction model: Coulomb plus viscous
%Obs: The hyperbolic tangent function models the friction profile
tau_Coulomb = p.friction(1:3,1);
b_viscous = p.friction(4:6,1);
friction = - diag(tau_Coulomb)*tanh(20*dq) - diag(b_viscous)*dq;

%Controller function that computes the joint torques to be applied
tau = fcn_controller(t,X,p);

%Solving for the joint accelerations
ddq = De \ (Be*tau + friction - Ce*dq - Ge); 

dXdt = [dq; ddq];
