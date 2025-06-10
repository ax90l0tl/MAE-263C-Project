% MAE263C-Jumping Leg simulation
% Author: Conrad Ku (Adapted from the HOPPY Simulator
% Last modified: 02/23/2021

clear all
close all
clc

% addpath gen
% addpath fcns

% --- parameters ---
% Get the parameters about the robot and simulation settings
p = get_params();
params = p.params;

N_jumps = 3;

dt = 0.01;
% Debugging Jump Controller
test.x = zeros(0); % debugging phase variable
test.F = zeros(3,0);
test.tau = zeros(3,0);



%Initial condition:
q0 = [0.35; -pi/4; pi/2];
dq0 = [0; 0; 0];
ic = [q0; dq0;]; 

% Checks if the Foot is already on the Ground
foot_init = fcn_p4(q0,params);
if foot_init(2)<0
    fprintf('Start Above the Floor! \n')
    return
end

% Time Lengths
t_start = 0;
t_jump = 2; % Time Per Jump
t_final = t_jump * (N_jumps+1);
% Data Logging Variable Initialization
t_out = t_start; % Simulation Time
X_out = ic'; % States
U_out = [ 0 0 ]; % Input Torques
F_out = [ 0 0 ]; % GRFs

p.time_contact = zeros(2,0); % Stores contact time in (1), then stance time in (2)
fprintf('Aerial Phase \n')
% Iterate Through # of Hops
for i=1:N_jumps+1
    if t_start == t_final
        fprintf('Simulation Ended \n')
        break
    end
    %Simulate unconstrained dynamics in the Aerial Phase
    options = odeset('Events',@(t,X)event_contact(t,X,p));
    [t,X] = ode45(@(t,X)robot_dyn_aerial(t,X,p),[t_start:dt:t_final, t_final], X_out(end,:),options); %, t_final
    % Stops upon Contact, outputs the time and states
    %--------------------------------------------------------------------------
    p.time_contact(:,end+1) = [t(end);0] ;
    p.pt_contact = fcn_p4(X(end,1:3),params);
    % Store Aerial Phase Variables
    n_timesteps = length(t); 
    t_out = [t_out; t(2:n_timesteps)];
    X_out = [X_out ;X(2:n_timesteps,:)];
    [~,U,F] = robot_dyn_aerial(t,X,p); % Collect Input Torques and Ground Forces
    U_out = [U_out ; U(2:n_timesteps,:)];
    F_out = [F_out ; F(2:n_timesteps,:)];
    t_start = t_out(end); % Log the new Start Time for the Next Phase
     % # try plotting up against task space 
    % Operational Space Control in the Air 
    p_out = zeros(length(t),2);
    for j=1:size(X,1)
        p_out(j,:) = fcn_p14(X(j,1:3),params)';
    end
    % Plot
    figure()
    plot(t,p_out(:,1),'r','LineWidth',2)
    hold on
    plot(t,p_out(:,2),'g','LineWidth',2)
    yline(p.foot_d(1),'--r','LineWidth',2)
    yline(p.foot_d(2),'--g','LineWidth',2)
    legend('X','Y')

    % Calculate the inelastic Impact Event
    X_minus = X_out(end,:);
    X_plus  = fcn_impact(X_minus,p);
    X_out(end,:) = X_plus'; %Rewrite value to Include Impact Force
    fprintf('Landing at %.3f s:\n',t(end))
    if i == N_jumps+1 % Have it just Stand and Hold Pose after Final Jump
        %Simulate Standing and Pose Maintenance 
        if t_start == t_final
            fprintf('Simulation Ended \n')
            break
        end
        [t,X] = ode45(@(t,X)robot_dyn_stance_stand(t,X,p),[t_start:dt:t_final,t_final],X_out(end,:));
        p.time_contact(2,end) = t(end);
        % Store Stance Phase Variables
        n_timesteps = length(t); 
        t_out = [t_out; t(2:n_timesteps)];
        X_out = [X_out ;X(2:n_timesteps,:)];
        [~,U,F] = robot_dyn_stance_stand(t,X,p); % Collect Input Torques and Ground Forces
        U_out = [U_out ; U(2:n_timesteps,:)];
        F_out = [F_out ; F(2:n_timesteps,:)];
        fprintf('Holding Pose! \n')
        fprintf('Simulation Ended at %.3f s\n', t(end))
    else % Jump
        if t_start == t_final
            fprintf('Simulation Ended \n')
            break
        end
        %Simulate constrained dynamics in the Stance Phase
        options = odeset('Events',@(t,X)event_flight(t,X,p)); 
        [t,X] = ode45(@(t,X)robot_dyn_stance_jump(t,X,p,test),[t_start:dt:t_final,t_final],X_out(end,:),options); % , t_final
        % p.time_LO = t(end); % Re-evaluate need/purpose
        p.time_contact(2,end) = t(end);
    
        % Store Stance Phase Variables
        n_timesteps = length(t); 
        t_out = [t_out; t(2:n_timesteps)];
        X_out = [X_out ;X(2:n_timesteps,:)];
        [~,U,F,test] = robot_dyn_stance_jump(t,X,p,test); % Collect Input Torques and Ground Forces
        U_out = [U_out ; U(2:n_timesteps,:)];
        F_out = [F_out ; F(2:n_timesteps,:)];
        t_start = t_out(end); % Log the new Start Time for the Next Phase
        if t(end)==t_final
            fprintf('Could not Achieve Jump \n')
        else
            fprintf('%d of %d Jumps Achieved! \n',i,N_jumps)
        end
    end

end

% Check if Joint Limits Violated
if any(X(:,2) > p.hip_upper) || any(X(:,2) < p.hip_lower)
    fprintf("Hip Joint Violates Joint Limits! \n")
end
if any(X(:,3) > p.knee_upper) || any(X(:,3) < p.knee_lower)
    fprintf("Knee Joint Violates Joint Limits! \n")
end
% Check for Torque Limit Violations
% Hip
for i=1:length(t_out)
    omega = X_out(i,5);
    tau = U_out(i,1);
    torque_constraints = p.A_motor * [omega;tau] - p.B_motor;
    if any(torque_constraints>0)
        fprintf('Hip Motor Torque Exceeds Limits at Time: %f s \n', t_out(i))
    end
end
% Knee
% Hip
for i=1:length(t_out)
    omega = X_out(i,6);
    tau = U_out(i,2);
    torque_constraints = p.A_motor * [omega;tau] - p.B_motor;
    if any(torque_constraints>0)
        fprintf('Knee Motor Torque Exceeds Limits at Time: %f s \n', t_out(i))
    end
end

figure()
plot(t_out,X_out(:,1))
hold on
pos_d = p.foot_d_stance;
stance_time = [t_start:0.25:t_final, t_final];
plot(stance_time,-pos_d(2)*ones(length(stance_time)),'--k','LineWidth',2)
title(['Hip Height Over ',num2str(N_jumps), ' Jumps'])


%Animating the robot:
% animateRobot(t_out,X_out,p,0)










