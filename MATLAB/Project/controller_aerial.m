function [u,F] = controller_aerial(t,X,p)
q = X(1:3,:);
dq = X(4:6,:);
params = p.params;
% Ensure that Unobservable States do not affect Controller
observability = [ 0 0 0 ;
                  0 1 0 ;
                  0 0 1 ; ] ;
q = observability * q;
dq = observability * dq;
% Control Gains in the Air (Task Space)
Kp = p.Kp_aerial;
Kd = p.Kd_aerial;

% Desired Foot Position w.r.t Base Link
% Make Sure within Feasible Workspace, No checks implemented yet
% -------------------------------
foot_d = [0 -0.05]'; 

% Forward Kinematics w.r.t. Base Link
foot = fcn_p14(q,params); % p1_4
% Velocity of the foot relative to the base, ignores the rising/falling of
% the base in the control calculations
J14 = fcn_J14(q,params);
v_foot =  J14 * dq; 

% Aerial Task Space PD Controller
% Change Control Law if Necessary
% -------------------------------
F = Kp * (foot_d-foot) - Kd * v_foot;
u = J14' * F;




