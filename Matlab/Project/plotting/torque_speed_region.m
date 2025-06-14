clc
close all
%Actuators reflected inertia matrix
N_KB = 9;
N_KBMB = 20;         % KBMB

%Actuators reflected inertia matrix
N_KB = 9;
N_KBMB = 20;         % KBMB

% Motor Parameters - At Output
kt = 1.16; % N-m/A
kv = 9; % RPM/V
kv = kv * 6;
% kv = kv * ((2*pi)/60); % rad/s/V 
R_armature = 0.6; % ohm
% ke =

% Motor Specs
torque_limit = 25; % N-m
max_voltage = 24; % V
max_current = 20;
no_load_speed = kv*max_voltage; % deg/s
stall_torque = kt * max_current;

A_motor = [ 0 , 1 ;
            0 , -1;
            stall_torque/no_load_speed , 1 ;
            -stall_torque/no_load_speed , -1 ; ];
B_motor = [torque_limit ; torque_limit ; stall_torque ; stall_torque];

% Plotting Aided by ChatGPT then edited

% Generate torque-speed polygon using polytope
omega = linspace(-no_load_speed*2.08, no_load_speed*2.08, 1000); % rad/s
tau_bounds = zeros(2, length(omega)); % [lower; upper] torque bounds

for i = 1:length(omega)
    w = omega(i);

    % Solve A*v <= B for each omega slice (1D LP to find tau bounds)
    % A*v = A*[w; tau] = a1*w + a2*tau <= b  =>  linear constraints on tau
    % Extract all allowable tau at this omega
    a1 = A_motor(:,1);
    a2 = A_motor(:,2);
    b = B_motor - a1 * w;

    % Feasible tau region at this omega
    tau_upper = min(b(a2 > 0) ./ a2(a2 > 0));
    tau_lower = max(b(a2 < 0) ./ a2(a2 < 0));

    % Store
    tau_bounds(1,i) = tau_lower;
    tau_bounds(2,i) = tau_upper;
end

% Plot the filled polygon with transparency
figure;
hold on;



% Plot individual constraints (dotted, different colors)
colors = lines(4);  
legend_entries = {};


plot(omega(1:477), torque_limit * ones(size(omega(1:477))), ':', 'LineWidth', 3, 'Color', colors(1,:));
legend_entries{end+1} = '\tau \leq limit';


plot(omega(520:end), -torque_limit * ones(size(omega(520:end))), ':', 'LineWidth', 3, 'Color', colors(2,:));
legend_entries{end+1} = '\tau \geq -limit';

slope = stall_torque / no_load_speed;
plot(omega(477:end), -slope * omega(477:end) + stall_torque, ':', 'LineWidth', 3, 'Color', colors(3,:));
legend_entries{end+1} = 'Upper Torque-Speed Limit';

plot(omega(1:520), -slope * omega(1:520) - stall_torque, ':', 'LineWidth', 3, 'Color', colors(4,:));
legend_entries{end+1} = 'Lower Torque-Speed Limit';

fill([omega fliplr(omega)], ...
     [tau_bounds(1,:) fliplr(tau_bounds(2,:))], ...
     [0.2 0.6 0.8], ...
     'EdgeColor', 'none', ...
     'FaceAlpha', 0.3); 
xline(0,"--k",'LineWidth',2)
yline(0,"--k",'LineWidth',2)

% Labels and appearance
% xlim([-50,50])
xlabel('Speed \omega (deg/s)');
ylabel('Torque \tau (N·m)');
title('Motor Torque-Speed Limit Polygon');
legend(legend_entries, 'Location', 'best');
grid on;
hold off;
%%
[tau_min,tau_max] = torque_bounds(2000)
%%
function [tau_min, tau_max] = torque_bounds(omega)
    % Constants
    kv = 9 * 6;           % deg/s/V
    no_load_speed = kv * 24;    % max voltage = 24 V
    stall_torque = 1.16 * 20;   % kt = 1.16 Nm/A, max current = 20 A

    % Polygon constraints: A*[omega; tau] <= B
    A = [ 0 1; 0 -1;
          stall_torque/no_load_speed 1;
         -stall_torque/no_load_speed -1 ];
    B = [23.2; 23.2; stall_torque; stall_torque];

    b = B - A(:,1) * omega;

    % Compute torque bounds
    up = min(b(A(:,2)>0) ./ A(A(:,2)>0,2));
    lo = max(b(A(:,2)<0) ./ A(A(:,2)<0,2));

    if isempty(up) || isempty(lo) || lo > up
        [tau_min, tau_max] = deal(NaN);
    else
        tau_min = lo;
        tau_max = up;
    end
end