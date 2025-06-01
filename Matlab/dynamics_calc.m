clc
clear
close all

% Define Frames

syms q1 q2 m1 m2 q1d q2d q1dd q2dd g
syms a1 a2
% a1 = 0.3;
% a2 = 0.25;
L(1) = Link('revolute','d', 0, 'a', 0, 'alpha', -pi/2 ,'modified', 'offset', pi/2);
L(2) = Link('revolute','d', 0, 'a', a1, 'alpha', 0, 'modified');
bot = SerialLink(L, 'name', 'RRR');
bot.tool = transl(a2, 0, 0);
% hold on
% trplot(trotx(-pi/2)*transl(0, 0, 0)*trotz(pi/2)*transl(0, 0, 0), 'frame', 'Feeder', 'length', 0.5, 'color', 'red', 'text_opts', {'FontSize', 8});
% bot.teach()
% simplify(bot.jacob0([q1, q2]))
simplify(bot.A(1, [q1, q2]))
simplify(bot.A(2, [q1, q2]))
simplify(bot.tool)

T_0_1 = simplify(trotx(-pi/2)*transl(0, 0, 0)*trotz(q1 + pi/2)*transl(0, 0, 0))
T_1_2 = trotx(0)*transl(a1, 0, 0)*trotz(q2)*transl(0, 0, 0)
T_2_T = trotx(0)*transl(a2, 0, 0)*trotz(0)*transl(0, 0, 0)

T_0_T = simplify(T_0_1*T_1_2*T_2_T)
T_0_2 = simplify(T_0_1*T_1_2);

subs(T_0_T, [a1, a2, q1, q2], [0.145, 0.215, -1.0, 2.62])

[R_0_1, P_0_1] = tr2rt(T_0_1);
R_1_0 = transpose(R_0_1);
[R_1_2, P_1_2] = tr2rt(T_1_2);
R_2_1 = transpose(R_1_2);
[R_2_T, P_2_T] = tr2rt(T_2_T);
R_T_2 = transpose(R_2_T);


[R_0_T, P_0_T] = tr2rt(T_0_T);
R_T_0 = transpose(R_0_T);
[R_0_2, P_0_2] = tr2rt(T_0_2);
R_2_0 = transpose(R_0_2);

%% Define center of mass positions and moment of inertia about COM
clc
syms Px1 Py1 Pz1 Px2 Py2 Pz2
PC1 = [Px1; Py1; Pz1];
PC2 = [Px2; Py2; Pz2];

syms I1xx I1yy I1zz I1xy I1yz I1xz
IC1 = [I1xx, I1xy, I1xz;
       I1xy, I1yy, I1yz;
       I1xz, I1yz, I1zz;];
syms I2xx I2yy I2zz I2xy I2yz I2xz
IC2 = [I2xx, I2xy, I2xz;
       I2xy, I2yy, I2yz;
       I2xz, I2yz, I2zz;];

th = [q1, q2];
thd = [q1d, q2d];
thdd = [q1dd, q2dd];
%% Newton Euler
clc
% Outward Iteration
% base frame isnt rotating
w0 = [0;0;0];
w0_d = [0;0;0];
% gravity vector
v0_d = [0; 0; -g];

% Link 1
w1 = R_1_0 * w0 + q1d*[0;0;1];
w1_d = R_1_0 * w0_d + cross(R_1_0*w0, q1d*[0;0;1]) + q1dd*[0;0;1];
v1_d = R_1_0 * (cross(w0_d, P_0_1) + cross(w0, cross(w0, P_0_1)) + v0_d);
v1_d_com = cross(w1_d, PC1) + cross(w1, cross(w1, PC1)) + v1_d;

F1 = m1*v1_d_com;
N1 = IC1 * w1_d + cross(w1, IC1*w1);

% Link 2
w2 = R_2_1 * w1 + q2d*[0;0;1];
w2_d = R_2_1 * w1_d + cross(R_2_1*w1, q2d*[0;0;1]) + q2dd*[0;0;1];
v2_d = R_2_1 * (cross(w1_d, P_1_2) + cross(w1, cross(w1, P_1_2)) + v1_d);
v2_d_com = simplify(cross(w2_d, PC2) + cross(w2, cross(w2, PC2)) + v2_d);

F2 = simplify(m2*v2_d_com);
N2 = simplify(IC2 * w2_d + cross(w2, IC2*w2));

% Inward Iteration
syms f_x f_y f_z n_x n_y n_z
% force and torque applied on end effector
f3 = [f_x;f_y;f_z];
n3 = [n_x;n_y;n_z];

f3 = [0;0;0;];
n3 = [0;0;0;];

% joint 2
f2 = simplify(R_2_T*f3 + F2);
n2 = simplify(N2 + R_2_T*n3 + cross(PC2, F2) + cross(P_2_T, R_2_T*f3));

tau2 = simplify(transpose(n2)*[0;0;1]);

% joint 1
f1 = simplify(R_1_2*f2 + F1);
n1 = simplify(N1 + R_1_2*n2 + cross(PC1, F1) + cross(P_1_2, R_1_2*f2));

tau1 = simplify(transpose(n1)*[0;0;1]);

tau = simplify([tau1;tau2]);

% Mass Component
mass_component = subs(tau, [q1, q1d, q2, q2d], [0, 0, 0, 0]);
mass_component(1)
mass_component(2)

% Gravity Component
gravity_component = subs(tau, [q1dd, q1d, q2dd, q2d], [0, 0, 0, 0]);
gravity_component(1);
gravity_component(2);

% Centrifugal Component
% centrifugal_and_coriolis = subs(tau, [t1dd, t2dd], [0, 0]) - gravity_component;
% centrifugal_and_coriolis(1)
% centrifugal_and_coriolis(2)
% Coriolis Component
%% Lagrange
% clc
% Link Inertia Matrix
I_0_1 = simplify(R_0_1*IC1*R_1_0);
I_0_2 = simplify(R_0_2*IC2*R_2_0);

P0c1 = T_0_1 * [PC1;1];
P0c1 = P0c1(1:3);
P0c2 = T_0_2 * [PC2;1];
P0c2 = P0c2(1:3);

% Link Jacobian Matrix
Jv1 = sym(zeros(3, 2));
Jv2 = sym(zeros(3, 2));
Jw1 = sym(zeros(3, 2));
Jw2 = sym(zeros(3, 2));

Jv1(1:3, 1) = cross(R_0_1(1:3, 3), (P0c1 - P_0_1));
Jw1(1:3, 1) = R_0_1(1:3, 3);

Jv2(1:3, 1) = cross(R_0_1(1:3, 3), (P0c2 - P_0_1));
Jw2(1:3, 1) = R_0_1(1:3, 3);
Jv2(1:3, 2) = cross(R_0_2(1:3, 3), (P0c2 - P_0_2));
Jw2(1:3, 2) = R_0_2(1:3, 3);

% Manipulator inertia matrix
M1 = simplify(transpose(Jv1)*m1*Jv1 + transpose(Jw1)*I_0_1*Jw1);
M2 = simplify(transpose(Jv2)*m2*Jv2 + transpose(Jw2)*I_0_2*Jw2);

M = M1 + M2


% Velocity coupling vector
V1 = sym(0); V2 = sym(0);

for j = 1:2
    for k = 1:2
        V1 = V1 + simplify((diff(M(1, j), th(k)) - 0.5*diff(M(j, k), q1)) * thd(j)*thd(k));
        V2 = V2 + simplify((diff(M(2, j), th(k)) - 0.5*diff(M(j, k), q2)) * thd(j)*thd(k));
    end
end
V = simplify([V1;V2;])

% Gravitational Effects
G1 = sym(0); G2 = sym(0);
m = [m1, m2];
Jv = {Jv1, Jv2};

for j = 1:2
    G1 = G1 - m(j)*[0, 0, g]*Jv{j}(1:3, 1);
    G2 = G2 - m(j)*[0, 0, g]*Jv{j}(1:3, 2);
end

G = simplify([G1;G2])

J = sym(zeros([6, 2]));
J(1:3, 1) = cross(R_0_1(1:3, 3), (P_0_T - P_0_1));
J(4:6, 1) = R_0_1(1:3, 3);
J(1:3, 2) = cross(R_0_2(1:3, 3), (P_0_T - P_0_2));
J(4:6, 2) = R_0_2(1:3, 3);
J = simplify(J)

dJ = sym(zeros(size(J)));
dq = [q1d, q2d];
q = [q1, q2];
for i = 1:length(q)
    dJ = dJ + diff(J, q(i)) * dq(i);  % Element-wise symbolic differentiation
end

simplify(dJ)

end_effector_force = simplify(R_0_T*f3);
end_effector_torque = simplify(R_0_T*n3);

joint_torque = simplify(transpose(J)*[end_effector_force;end_effector_torque]);

tau_lagrange = simplify(simplify(M*transpose(thdd)) + V + G + joint_torque);
tau_lagrange(1);

expand(tau_lagrange(1)) - expand(tau1);
expand(tau_lagrange(2)) - expand(tau2);

subs(M, [a1, a2, q1, q2, I1zz, I2zz, Px1, Py1, Px2, Py2, m1, m2], [0.145, 0.215, -1.0, 2.62, 0.01275, 0.00407, 0.016172, 0.057426, 0.119035, -0.021572, 0.5, 0.07])