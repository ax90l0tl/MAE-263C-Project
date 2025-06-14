function f = fcn_disturbance(t,p)%,X,p) % Applied Step Force to the Base
% params = p.params;
% q = X(1:3);
% J2t = fcn_J2(q,params)';
magnitude = 20;
start_time = 0.5;
step_duration = 1;


if (start_time<=t) && (t<start_time+step_duration)
    f = [0 , magnitude]';
elseif (2*start_time+step_duration<=t) && (t< 2*start_time + 2*step_duration)
    f = [0 , -magnitude]';
else
    f = [0 0]';
end
end



