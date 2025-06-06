function X_plus = fcn_impact(X,p)
q_minus = X(1:3)';
dq_minus = X(4:6)';
params = p.params;

M = fcn_Me(q_minus,params);
Jhc = fcn_Jhc(q_minus,params); % Foot Relative to Hip, Hip Frame (I could've called the wrong variable)

% Impact Map Equations A * [ dq_plus ; F_impact ] = b
% M * (dq_plus - dq_minus) = J' * F_impact % Change in Momentum, i.e. Inelastic Collision
% J * dq_plus = 0   % Velocity of the Contact Point set to Zero
A_mat = [ M , -Jhc' ;
          Jhc, zeros(2,2)];
B_vec = [ M * dq_minus ; zeros(2,1) ];
sol_vec = A_mat \ B_vec;

X_plus = [ q_minus ; sol_vec(1:3) ]';
F_imp = sol_vec(4:5)';

end