% generate the dynamics for the hopping leg with boom
% Author: Yanran Ding
% Last modified: 2020/1/19
clear all

%% --- define symbols ---
syms q1 q2 q3 real
syms dq1 dq2 dq3 real
syms l1 l2 l3 real
syms M1 M2 M3 real
syms J1_xx J1_yy J1_zz J1_xy J1_xz J1_yz real 
syms J2_xx J2_yy J2_zz J2_xy J2_xz J2_yz real
syms J3_xx J3_yy J3_zz J3_xy J3_xz J3_yz real
syms g real

%% --- variable lists ---
% parameters
m_list_params = {
    'g'  'p(1)'
    'l1' 'p(2)';
    'l2' 'p(3)';
    'l3' 'p(4)';
    'M1' 'p(5)';
    'M2' 'p(6)';
    'M3' 'p(7)';
    'J1_xx' 'p(8)';
    'J1_yy' 'p(9)';
    'J1_zz' 'p(10)';
    'J1_xy' 'p(11)';
    'J1_xz' 'p(12)'; 
    'J1_yz' 'p(13)';
    'J2_xx' 'p(14)'; 
    'J2_yy' 'p(15)';
    'J2_zz' 'p(16)'; 
    'J2_xy' 'p(17)';
    'J2_xz' 'p(18)'; 
    'J2_yz' 'p(19)';
    'J3_xx' 'p(20)'; 
    'J3_yy' 'p(21)'; 
    'J3_zz' 'p(22)'; 
    'J3_xy' 'p(23)'; 
    'J3_xz' 'p(24)'; 
    'J3_yz' 'p(25)';};

% joint position
m_list_q = {
    'q1' 'q(1)';
    'q2' 'q(2)';
    'q3' 'q(3)'};

% joint velocity
m_list_dq = {
    'dq1' 'dq(1)';
    'dq2' 'dq(2)';
    'dq3' 'dq(3)'};


%% --- variables ---
q = [q1 q2 q3]';
dq = [dq1 dq2 dq3]';

%% --- forward kinematics ---
% 0 - origin
% 1 - base
% 2 - shoulder
% 3 - elbow
% 4 - end-effector

T01 = [rz(q1) [0 0 0]';
       0, 0, 0, 1];

Mtemp = [1 0 0; 0 0 1; 0 -1 0];
T12 = [Mtemp*rz(q2) [0 0 l1]';
       0, 0, 0, 1];

T23 = [rz(q3) [l2 0 0]';
       0, 0, 0, 1];

T34 = [eye(3) [l3 0 0]';
       0, 0, 0, 1];


%% joint position
T02 = T01 * T12;
p2 = T02(1:3,4);
write_fcn_m('fcn_p2.m',{'q','p'},[m_list_q;m_list_params],{p2,'p2'});

T03 = T01 * T12 * T23;
p3 = T03(1:3,4);
write_fcn_m('fcn_p3.m',{'q','p'},[m_list_q;m_list_params],{p3,'p3'});

T04 = T03 * T34;
p4 = T04(1:3,4);
J4 = jacobian(p4,q);
write_fcn_m('fcn_p4.m',{'q','p'},[m_list_q;m_list_params],{p4,'p4'});
write_fcn_m('fcn_J4.m',{'q','p'},[m_list_q;m_list_params],{J4,'J4'});

dJ4 = sym('dJ4',size(J4));
for ii = 1:size(dJ4,2)
    dJ4(:,ii) = jacobian(J4(:,ii),q) * dq;
end
write_fcn_m('fcn_dJ4.m',{'q','dq','p'},[m_list_q;m_list_dq;m_list_params],{dJ4,'dJ4'});


%% Center of Mass pos/vel
T1com = [eye(3) [0 0 l1/2]';
         0, 0, 0, 1];
     
T2com = [eye(3) [l2/2 0 0]';
         0, 0, 0, 1];
     
T3com = [eye(3) [l3/2 0 0]';
         0, 0, 0, 1];
     
T01com = T01 * T1com;
R_1com = T01com(1:3,1:3);           % R_icom: CoM Rot mat of the ith link
p_1com = T01com(1:3,4);             % p_icom: CoM pos of the ith link
v_1com = jacobian(p_1com,q) * dq;   % v_icom: CoM vel of the ith link

T02com = T01 * T12 * T2com;
R_2com = T02com(1:3,1:3);
p_2com = T02com(1:3,4);
v_2com = jacobian(p_2com,q) * dq;

T03com = T01 * T12 * T23 * T3com;
R_3com = T03com(1:3,1:3);
p_3com = T03com(1:3,4);
v_3com = jacobian(p_3com,q) * dq;


%% angular velocity
k1_hat = [0 0 1]';
k2_hat = [0 0 1]';
k3_hat = [0 0 1]';

Jw = [R_1com*k1_hat, R_2com*k2_hat, R_3com*k3_hat];

% w around CoM in the world frame
w1_s = Jw(:,1) * dq(1);
w2_s = Jw(:,1:2) * dq(1:2);
w3_s = Jw(:,1:3) * dq(1:3);


%% --- Energy and Lagrangian ---
I1 = [[J1_xx J1_xy J1_xz]; [J1_xy J1_yy J1_yz]; [J1_xz J1_yz J1_zz];];
I2 = [[J2_xx J2_xy J2_xz]; [J2_xy J2_yy J2_yz]; [J2_xz J2_yz J2_zz];];
I3 = [[J3_xx J3_xy J3_xz]; [J3_xy J3_yy J3_yz]; [J3_xz J3_yz J3_zz];];

KE_1 = 0.5 * v_1com' * M1 * v_1com + 0.5 * w1_s' * (R_1com*I1*R_1com') * w1_s;
KE_2 = 0.5 * v_2com' * M2 * v_2com + 0.5 * w2_s' * (R_2com*I2*R_2com') * w2_s;
KE_3 = 0.5 * v_3com' * M3 * v_3com + 0.5 * w3_s' * (R_3com*I3*R_3com') * w3_s;

% Kinetic energy
KE = simplify(KE_1 + KE_2 + KE_3);   

% potential energy
PE = g * (M1*p_1com(3) + M2*p_2com(3) + M3*p_3com(3));

Upsilon = [q1 q2 q3]; % where control torques go

%% --- Euler-Lagrange Equation ---
[De, Ce, Ge, Be] = std_dynamics(KE,PE,q,dq,Upsilon);

write_fcn_m('fcn_De.m',{'q', 'p'},[m_list_q;m_list_params],{De,'De'});
write_fcn_m('fcn_Ce.m',{'q', 'dq', 'p'},[m_list_q;m_list_dq;m_list_params],{Ce,'Ce'});
write_fcn_m('fcn_Ge.m',{'q', 'p'},[m_list_q;m_list_params],{Ge,'Ge'});
write_fcn_m('fcn_Be.m',{'q', 'p'},[m_list_q;m_list_params],{Be,'Be'});











