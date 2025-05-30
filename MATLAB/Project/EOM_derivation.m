clc; clear all; close all;
sympref('FloatingPointOutput',true);
format short
%% Construct Arm
syms q1(t) q2(t) % time varying joint parameters
q = [q1(t) , q2(t)];
syms q1d(t) q2d(t) 
qd = [q1d(t) , q2d(t)];
syms q1dd(t) q2dd(t)
qdd = [q1dd(t) , q2dd(t)];



syms L1 L2
L = [L1 , L2];


% DH Parameters and FK
DH_table = [ 0 , q1(t) , 0    , 0         ;
             0 , q2(t) , L(1) , 0         ;
             0 , 0         , L(2) , 0         ;];

T.T01 = DH_Matrix(DH_table(1,:));
T.T12 = DH_Matrix(DH_table(2,:));
T.T23 = DH_Matrix(DH_table(3,:));

T.T02 = simplify(T.T01 * T.T12);
T.T03 = simplify(T.T01 * T.T12 * T.T23);
T_end = T.T03; 
P1_function = @(Q) subs(T.T01(1:3,4), q, Q');
P2_function = @(Q) subs(T.T02(1:3,4), q, Q');
P3_function = @(Q) subs(T.T03(1:3,4), q, Q');

% Hard-Coding Stuff for Sanity
R01 = T.T01(1:3,1:3);
R12 = T.T12(1:3,1:3);
R23 = T.T23(1:3,1:3);
R10 = R01.';
R21 = R12.';
R32 = R23.';
R03 = T.T03(1:3,1:3);
R30 = R03.';
P01 = T.T01(1:3,4);
P12 = T.T12(1:3,4);
P23 = T.T23(1:3,4);


%% Lagrange Formulation
syms m1 m2 M_base
m = [m1 , m2, M_base];
syms I1 I2
Iz = [I1, I2];
syms g
syms Lc1 Lc2

T0COM1 = T.T01*transl([Lc1,0,0]);
T0COM2 = T.T02*transl([Lc2,0,0]);
P0COM1 = T0COM1(1:3,4);
P0COM2 = T0COM2(1:3,4);
P03    = T.T03(1:3,4);


V0COM1 = subs(diff(P0COM1,t),diff(q,t),qd);
W0COM1 = q1d(t);
K1 = simplify(0.5 * m(1) * (V0COM1.'*V0COM1) + 0.5 * Iz(1) * W0COM1^2);
P1 = simplify(m(1) * g * P0COM1(2));


V0COM2 = subs(diff(P0COM2,t),diff(q,t),qd);
W0COM2 = q1d(t)+q2d(t);

K2 = simplify(0.5 * m(2) * (V0COM2.'*V0COM2) + 0.5 * Iz(2) * W0COM2^2);
P2 = simplify(m(2) * g * P0COM2(2));

V03 = subs(diff(P03,t),diff(q,t),qd);
K3 =  simplify(0.5 * m(3) * (V03.'*V03));
P3 =  simplify(m(3) * g * P03(2));

K = K1+K2+K3;
P = (P1 + P2 + P3);
LG = K - P;
EOM = [ subs(diff(diff(K,qd(1)),t),[diff(qd,t),diff(q,t)],[qdd,qd]) - diff(K,q(1)) + diff(P,q(1)); 
        subs(diff(diff(K,qd(2)),t),[diff(qd,t),diff(q,t)],[qdd,qd]) - diff(K,q(2)) + diff(P,q(2));    ];
       

[M, n] = equationsToMatrix(EOM,qdd);
M  = simplify(M)
[G,B] = equationsToMatrix(n,g);
B = simplify(B)
G = simplify(G*g)


% % J_ext = [ -L(1)*sin(q(1)) - L(2)*sin(q(1)+q(2)) , -L(2)*sin(q(1)+q(2));
% %            L(1)*cos(q(1)) + L(2)*cos(q(1)+q(2)) ,  L(2)*cos(q(1)+q(2));
% %            1                                    ,  1                  ;];

%% DH Matrix
function T = DH_Matrix(dh)
    alpha = dh(1);
    theta = dh(2);
    a     = dh(3);
    d     = dh(4); 
    T = [ cos(theta)            , -sin(theta)            ,  0          ,  a             ;
          sin(theta)*cos(alpha) ,  cos(theta)*cos(alpha) , -sin(alpha) , -d * sin(alpha);
          sin(theta)*sin(alpha) ,  cos(theta)*sin(alpha) ,  cos(alpha) ,  d * cos(alpha);
          0                     ,  0                     ,  0          ,  1             ;];
end
