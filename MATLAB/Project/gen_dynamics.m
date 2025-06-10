% Generate the Dynamics for the Leg
% Adapted from Yanran Ding's Script
clear all
syms q1 q2 q3 real
syms dq1 dq2 dq3 real
syms l1 l2 l3 real
syms M1 M2 M3 real
syms J1_xx J1_yy J1_zz J1_xy J1_xz J1_yz real 
syms J2_xx J2_yy J2_zz J2_xy J2_xz J2_yz real
syms J3_xx J3_yy J3_zz J3_xy J3_xz J3_yz real
syms g real
syms X Y real
syms l2comx l2comy l3comx l3comy real

%% Variable List

% Params
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
    'J3_yz' 'p(25)';
    'l2comx' 'p(26)'
    'l2comy' 'p(27)'
    'l3comx' 'p(28)'
    'l3comy' 'p(29)'};

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

% task space position
m_list_p = {
    'X' 'pos(1)';
    'Y' 'pos(2)'};

%% Joint Variables
q = [q1 q2 q3]'; % height, theta1, theta2
dq = [dq1 dq2 dq3]';

%% IK - 2D [From Base to Foot]
pos = [X ; Y];

length_int = norm(pos);
theta_2 = pi - acos((l2^2 + l3^2 - length_int^2)/(2*l2*l3));
theta_1 = atan2( Y , X ) - atan2(l3*sin(theta_2),(l2+l3*cos(theta_2))) + pi/2;
assume([X Y l2 l3],'real'); 
q_IK = [ theta_1 ; theta_2 ];
q_IK = simplify(q_IK,'Steps',50);
write_fcn_m('fcn_IK.m',{'pos','p'},[m_list_p;m_list_params],{q_IK,'q_IK'});

%% FK - 2D

T01 = [eye(2), [0,q1]';
       0 0 1];
T12 = [rot(q2-pi/2), [l1,0]';
       0 0 1];
T23 = [rot(q3), [l2,0]';
       0 0 1];
T34 = [eye(2) , [l3,0]';
       0 0 1];
write_fcn_m('fcn_T01.m',{'q','p'},[m_list_q;m_list_params],{T01,'T01'});
write_fcn_m('fcn_T12.m',{'q','p'},[m_list_q;m_list_params],{T12,'T12'});
write_fcn_m('fcn_T23.m',{'q','p'},[m_list_q;m_list_params],{T23,'T23'});
write_fcn_m('fcn_T34.m',{'q','p'},[m_list_q;m_list_params],{T34,'T34'});
% Joint Positions

T02 = T01 * T12;
p2 = T02(1:2,3);
J2 = jacobian(p2,q);
write_fcn_m('fcn_p2.m',{'q','p'},[m_list_q;m_list_params],{p2,'p2'});
write_fcn_m('fcn_J2.m',{'q','p'},[m_list_q;m_list_params],{J2,'J2'});

T03 = T02 * T23;
p3 = T03(1:2,3);
write_fcn_m('fcn_p3.m',{'q','p'},[m_list_q;m_list_params],{p3,'p3'});

T04 = T03 * T34;
p4 = T04(1:2,3);
J4 = jacobian(p4,q);
write_fcn_m('fcn_p4.m',{'q','p'},[m_list_q;m_list_params],{p4,'p4'});
write_fcn_m('fcn_J4.m',{'q','p'},[m_list_q;m_list_params],{J4,'J4'});

% Foot w.r.t to Base of Prismatic
T14 = T12*T23*T34;
p14 = T14(1:2,3);
J14 = jacobian(p14,q);
write_fcn_m('fcn_p14.m',{'q','p'},[m_list_q;m_list_params],{p14,'p14'});
write_fcn_m('fcn_J14.m',{'q','p'},[m_list_q;m_list_params],{J14,'J14'});

% Foot w.r.t to Hip Joint
T24 = T23 * T34;
p24 = T24(1:2,3);
J24 = jacobian(p24,q);
write_fcn_m('fcn_p24.m',{'q','p'},[m_list_q;m_list_params],{p24,'p24'});
write_fcn_m('fcn_J24.m',{'q','p'},[m_list_q;m_list_params],{J24,'J24'});


% Jacobian Derivative 
dJ4 = sym('dJ4',size(J4));
for ii = 1:size(dJ4,2)
    dJ4(:,ii) = jacobian(J4(:,ii),q) * dq;
end
write_fcn_m('fcn_dJ4.m',{'q','dq','p'},[m_list_q;m_list_dq;m_list_params],{dJ4,'dJ4'});

%% Foot Constraints

% Point of Contact is the toe position at impact time
point_contact = [0 , 0 , 0]';
z_hat = [ 0 0 1 ]';
y_hat = [ 0 1 0 ]';
x_hat = [ 1 0 0 ]';
R = [ x_hat y_hat z_hat];

%%
% The Holonomic constraints are:
% foot position x and y in Toe Frame do not change during stance
hol_con = p4([1,2]);
Jhc = jacobian(hol_con,q);
dJhc = sym('dJhc',size(Jhc));
for ii = 1:size(Jhc,2)
    dJhc(:,ii) = jacobian(Jhc(:,ii),q) * dq;
end
write_fcn_m('fcn_phc.m',{'q','p'},[m_list_q;m_list_params],{hol_con,'phc'})
write_fcn_m('fcn_Jhc.m',{'q','p'},[m_list_q;m_list_params],{Jhc,'Jhc'});
write_fcn_m('fcn_dJhc.m',{'q', 'dq', 'p'},[m_list_q;m_list_dq;m_list_params],{dJhc,'dJhc'});

%% COM Kinematics
T1com = [eye(2), [l1/2,0]';
       0 0 1];
T2com = [eye(2), [l2comx,l2comy]';
       0 0 1];
T3com = [eye(2) , [l3comx,l3comy]';
       0 0 1];

T01com = T01*T1com;
R01com = T01com(1:2,1:2);
p01com = T01com(1:2,3);
v01com = jacobian(p01com,q) * dq;

T02com = T01*T12*T2com;
R02com = T02com(1:2,1:2);
p02com = T02com(1:2,3);
v02com = jacobian(p02com,q) * dq;

T03com = T01*T12*T23*T3com;
R03com = T03com(1:2,1:2);
p03com = T03com(1:2,3);
v03com = jacobian(p03com,q) * dq;

%% Angular Velocities
% Rotation Axis
k1 = [0 0 0]';
k2 = [0 0 1]';
k3 = [0 0 1]';

Jw = [ [R01com [0 ,0]'; 0 0 1]*k1 , [R02com [0 ,0]'; 0 0 1]*k2 , [R03com [0 ,0]'; 0 0 1]*k3 ];

omega1 = Jw(:,1) * dq(1);
omega2 = Jw(:,1:2) * dq(1:2);
omega3 = Jw(:,1:3) * dq(1:3);

%% Energy
% Inertia of Links
I1 = [[J1_xx J1_xy J1_xz]; [J1_xy J1_yy J1_yz]; [J1_xz J1_yz J1_zz];];
I2 = [[J2_xx J2_xy J2_xz]; [J2_xy J2_yy J2_yz]; [J2_xz J2_yz J2_zz];];
I3 = [[J3_xx J3_xy J3_xz]; [J3_xy J3_yy J3_yz]; [J3_xz J3_yz J3_zz];];

R01com3d = [R01com [0 ,0]'; 0 0 1];
R02com3d = [R02com [0 ,0]'; 0 0 1];
R03com3d = [R03com [0 ,0]'; 0 0 1];

KE_1 = 0.5 * v01com' * M1 * v01com + 0.5 * omega1' * (R01com3d*I1*R01com3d') * omega1;
KE_2 = 0.5 * v02com' * M2 * v02com + 0.5 * omega2' * (R02com3d*I2*R02com3d') * omega2;
KE_3 = 0.5 * v03com' * M3 * v03com + 0.5 * omega3' * (R03com3d*I3*R03com3d') * omega3;

KE = simplify(KE_1 + KE_2 + KE_3);

PE = g * ( M1 * p01com(2) + M2 * p02com(2) + M3 * p03com(2));

upsilon = [0 q2 q3];

%% Euler Lagrange Function
[Me, Ce, Ge, Be] = std_dynamics(KE,PE,q,dq,upsilon);

write_fcn_m('fcn_Me.m',{'q', 'p'},[m_list_q;m_list_params],{Me,'Me'});
write_fcn_m('fcn_Ce.m',{'q', 'dq', 'p'},[m_list_q;m_list_dq;m_list_params],{Ce,'Ce'});
write_fcn_m('fcn_Ge.m',{'q', 'p'},[m_list_q;m_list_params],{Ge,'Ge'});
write_fcn_m('fcn_Be.m',{'q', 'p'},[m_list_q;m_list_params],{Be,'Be'});

%% Change of Base Grav Comp ( From Eric )
% l3comx_stance = l3-l3comx;
% l3comy_stance = l3comy;
% l2comx_stance = l2-l2comx;
% l2comy_stance = l2comy;
% l2com_stance = sqrt(l2comx_stance^2 + l2comy_stance^2);
% q1_stance = pi/2 + q2 + q1;
% q2_stance = -q2;
% FK_hip = [l3*cos(q1_stance) + l2*cos(q1_stance+q2_stance) ;
%           l3*sin(q1_stance) + l2*sin(q1_stance+q2_stance)];
% FK_tibia_com = [l3comx_stance*cos(q1_stance);
%                l3comx_stance*sin(q1_stance);];
% FK_femur_com = [l3*cos(q1_stance) + l2comx_stance*cos(q1_stance+q2_stance);
%                 l3*sin(q1_stance) + l2comx_stance*sin(q1_stance+q2_stance); ];
% G_comp_stance = [0;
%     -(M3*g*l3comx_stance*cos(q1_stance) + M1*g*(l2*cos(q1_stance+q2_stance)+l3*cos(q1_stance)) +M2*g*(l2com_stance*cos(q1_stance+q2_stance-hip_angle)+l3*cos(q1_stance)));
%     -(M1+M2+M3)*g*l3*cos(q1+q2) + M3*g*l3comx*cos(q1+q2)];



%% Rotation Functions
function R = rot(theta)
R = [cos(theta)     -sin(theta)   ;
     sin(theta)    cos(theta)  ];

end
