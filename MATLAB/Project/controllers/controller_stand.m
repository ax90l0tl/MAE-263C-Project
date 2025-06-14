function [u,test] = controller_stand(t,X,p,test)
params = p.params;
q = X(1:3);
dq = X(4:6);
G = fcn_Ge(q,params);

% Ensure that Unobservable States do not affect Controller
observability = [ 0 0 0 ;
                  0 1 0 ;
                  0 0 1 ; ] ;
q = observability * q;
dq = observability * dq;
% Control Gains in the Ground (Task Space)
Kp = p.Kp_stance;
Kd = p.Kd_stance;


% Feedback Torque using Controller to Stabilize System About Set Point in Task Space
x_des = p.foot_d_stance; % Desired Task Space Position of the Foot w.r.t Base
x = fcn_p14(q,params);
% Force Feedforward (Static Weight)
f_ff = [0;p.weight];
% Jacobian from Hip to Foot
J14 = fcn_J14(q,params);
x_dot = J14*dq;
% q_des = fcn_IK(x_des,params);
f_pd = Kp * (x_des - x) - Kd * x_dot;
u = (J14' * f_pd) -(J14' * f_ff) + G; % Gravity Compensation to Reduce SS Errors


