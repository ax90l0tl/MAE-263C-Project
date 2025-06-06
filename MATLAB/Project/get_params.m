function p = get_params()
% Place to Pre-Load Global Static Variables, as well as pass them in and
% out of functions

% parameters for matrix calculations

% g Gravity constant
% l1,l2,l3 Length of link 1-3
% M1,M2,M3 Mass of link 1-3
% J1,J2,J3 Moment of Inertia of Link 1-3

g = 9.81; %m/s^2

l1 = 0; % unit: meter
l2 = 0.05;
l3 = l2;

M1 = 5; % unit: Kilogram
M2 = 3;
M3 = 2;

p.time_stance = 0.25;
p.jump_force = 1.5 * (M1+M2+M3)*g;

p.threshold = 0.025 * sum(M1 + M2 + M3) * g;

% Gains Matrices
%       Stance Phase
p.Kp_stance = diag(10 * [1 1]);
p.Kd_stance = diag(1 * [1 1]);
%       Jump   Phase
p.Kp_jump = diag(2.5 * [1 1]);
p.Kd_jump = diag(0.25 * [1 1]);

%       Aerial Phase
p.Kp_aerial = diag(10 * [1 1]);
p.Kd_aerial = diag(1 * [1 1]);



%Links inertia tensors (Note: J_ij = J_ji)
%Link 1
%I1 = [[J1_xx J1_xy J1_xz];
%      [J1_xy J1_yy J1_yz];
%      [J1_xz J1_yz J1_zz];]; kg*m^2
       
J1_xx = 0; 
J1_yy = 0; 
J1_zz = 1; 
J1_xy = 0; 
J1_xz = 0; 
J1_yz = 0;

%Link 2
J2_xx = 0; 
J2_yy = 0; 
J2_zz = 2; 
J2_xy = 0; 
J2_xz = 0; 
J2_yz = 0;

%Link 3
J3_xx = 0; 
J3_yy = 0; 
J3_zz = 0.5; 
J3_xy = 0; 
J3_xz = 0; 
J3_yz = 0;

%Friction model Coulomb + viscous
tau1_Coulomb = 0;
tau2_Coulomb = 0;
tau3_Coulomb = 0;
b1_viscous = 0;
b2_viscous = 0;
b3_viscous = 0;

params = [g, l1, l2, l3, M1, M2, M3, J1_xx, J1_yy, J1_zz, J1_xy, J1_xz,...
    J1_yz, J2_xx, J2_yy, J2_zz, J2_xy, J2_xz, J2_yz, J3_xx, J3_yy, J3_zz,...
    J3_xy, J3_xz, J3_yz];

p.friction = [tau1_Coulomb; tau2_Coulomb; tau3_Coulomb; b1_viscous; b2_viscous; b3_viscous];
p.params = params;
p.N_animate = 30;   % for animation time smoothness
















