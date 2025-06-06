% 3-link manipulator simulation
% Author: Yanran Ding and Joao Ramos
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

N_jumps = 1;
p.n_jumps = N_jumps;

dt = 0.01;
p.x = 0; % debugging phase variable

%Actuators reflected inertia matrix
N = 20;         % KBMB
Ir = 1.82e-3;   % From KB, assumed same rotor
M_DH = [[N^2*Ir 0 0]; [0 2*N^2*Ir N^2*Ir]; [0 N^2*Ir N^2*Ir];];

%Initial condition:
q0 = [1; -pi/4; pi/2];     
dq0 = [0; 0; 0];
ic = [q0; dq0;]; 

% Time Lengths
t_start = 0;
t_final = 5 * N_jumps;
% Data Logging Variable Initialization
t_out = t_start; % Simulation Time
X_out = ic'; % States
U_out = [ 0 0 ]; % Input Torques
F_out = [ 0 0 ]; % GRFs

p.time_contact = zeros(2,0); % Stores contact time in (1), then stance time in (2)

% Iterate Through # of Hops
for i=1:N_jumps
    %Simulate unconstrained dynamics in the Aerial Phase
    options = odeset('Events',@(t,X)event_contact(t,X,p));
    [t,X] = ode45(@(t,X)robot_dyn_aerial(t,X,p),[t_start:dt:t_final, t_final], X_out(end,:),options);
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
    
    % Calculate the inelastic Impact Event
    X_minus = X_out(end,:);
    X_plus  = fcn_impact(X_minus,p);
    X_out(end,:) = X_plus'; %Rewrite value to Include Impact Force

    if i == N_jumps+1 % Have it just Stand and Hold Pose after Final Jump
        %Simulate Standing and Pose Maintenance 
        [t,X] = ode45(@(t,X)robot_dyn_stance_stand(t,X,p),[t_start t_final],X_out(end,:));
        p.time_contact(2,end) = t(end);
    
        % Store Stance Phase Variables
        n_timesteps = length(t); 
        t_out = [t_out; t(2:n_timesteps)];
        X_out = [X_out ;X(2:n_timesteps,:)];
        [~,U,F] = robot_dyn_stance_stand(t,X,p); % Collect Input Torques and Ground Forces
        U_out = [U_out ; U(2:n_timesteps,:)];
        F_out = [F_out ; F(2:n_timesteps,:)];
        t_start = t_out(end); % Log the new Start Time for the Next Phase
    else % Jump
        %Simulate constrained dynamics in the Stance Phase
        options = odeset('Events',@(t,X)event_flight(t,X,p)); 
        [t,X] = ode45(@(t,X)robot_dyn_stance_jump(t,X,p),[t_start:dt:t_final, t_final],X_out(end,:),options);
        % p.time_LO = t(end); % Re-evaluate need/purpose
        p.time_contact(2,end) = t(end);
    
        % Store Stance Phase Variables
        n_timesteps = length(t); 
        t_out = [t_out; t(2:n_timesteps)];
        X_out = [X_out ;X(2:n_timesteps,:)];
        [~,U,F] = robot_dyn_stance_jump(t,X,p); % Collect Input Torques and Ground Forces
        U_out = [U_out ; U(2:n_timesteps,:)];
        F_out = [F_out ; F(2:n_timesteps,:)];
        t_start = t_out(end); % Log the new Start Time for the Next Phase
    end
    fprintf('%d of %d Steps Done! \n',i,N_jumps)
end

%Animating the robot:
animateRobot(t_out,X_out,p)










