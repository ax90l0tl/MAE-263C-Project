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

%Initial conditions:
q0 = [0.1; 0; 0];
dq0 = [0; 0; 0];
ic = [q0; dq0];

%Plot the robot at initial condition:
% figure
% plotRobot(ic,p);

% Simulate
global error 
error = 0;
tfinal = 5;
[t,X] = ode45(@(t,X)robot_dyn(t,X,p),[0 tfinal],ic);
X = X'; 

%Reconstructing control inputs and desired trajectory for plotting
for ii = 1:length(t)
    tau(:,ii) = fcn_controller(t(ii),X(:,ii),p);
    % [qd(:,ii) , qd_dot(:,ii) , qd_dotdot(:,ii) ] = fcn_trajectory(t(ii));
end

% figure()
% subplot(3,1,1),plot(t,X(1:3,:),'LineWidth',2)
% ylabel('Angles q [rad]')
% legend('\theta_1','\theta_2','\theta_3')
% grid on
% subplot(3,1,2),plot(t,X(4:6,:),'LineWidth',2)
% ylabel('Velocity dq/dt [rad/s]')
% legend('d\theta_1t/d','d\theta_2/dt','d\theta_3/dt')
% grid on
% subplot(3,1,3),plot(t,tau,'LineWidth',2)
% ylabel('Joint torque \tau [Nm]')
% xlabel('Time [s]')
% legend('\tau_1','\tau_2','\tau_3')
% grid on
% 
% figure
% plot(t,X(1:3,:)-qd,'LineWidth',2)
% ylabel('Joing angle error e(t) = q_d(t) - q(t) [rad]')
% xlabel('Time [s]')
% legend('e_1','e_2','e_3')
% grid on

%Animating the robot:
animateRobot(t,X,p)










