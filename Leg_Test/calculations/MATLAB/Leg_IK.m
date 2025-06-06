clear all; clc; close all;
%% Define Robot
fprintf('* ========== DH with base on joint 1 ========== *\n')

syms l1 l2
L1 = Link('revolute','d',0,'a',0,'alpha',0,'modified');
L2 = Link('revolute','d',0,'a',l1,'alpha',0,'modified');
L3 = Link('revolute','d',0,'a',l2,'alpha',0,'modified');
Leg_DH = SerialLink([L1 L2 L3], 'name', '2D Robot Leg')

%% Forward Kinematics: Robotname.fkine()
syms t1 t2;
t3 = 0;
th = [t1, t2, t3];
FK1 = Leg_DH.fkine(th);


%% IK
L1 = 125;
L2 = 215;

p = [159.28, -43.93];
q = IK(p, L1, L2);
q_deg = rad2deg(q);

%% New FK where the foot is now the base frame
fprintf('* ========== DH with base on foot ========== *')

syms l1 l2
L1 = Link('revolute','d',0,'a',0,'alpha',0,'modified');
L2 = Link('revolute','d',0,'a',l2,'alpha',0,'modified');
L3 = Link('revolute','d',0,'a',l1,'alpha',0,'modified');
Leg_DH = SerialLink([L1 L2 L3], 'name', '2D Robot Leg')

syms t1 t2;
t3 = 0;
th = [t1, t2, t3];
FK1 = Leg_DH.fkine(th)