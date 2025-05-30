function tau = fcn_controller(t,X,p)

%Robot parameters
params = p.params;

%Current states 
q = X(1:3);     %joint angles
dq = X(4:6);    %joints velocities


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIGN YOUR CONTROLLER HERE!

%Compute desired trajecotry
[qd qd_dot qd_dotdot] = fcn_trajectory(t);

%Controller
global error
error = error + (qd - q);
tau = ...

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Control saturation - DO NOT CHANGE
taumax = 30; %Maximum torque in [Nm] that can be applied at each joint
tau(tau>taumax) = taumax;
tau(tau<-taumax) = -taumax;


end