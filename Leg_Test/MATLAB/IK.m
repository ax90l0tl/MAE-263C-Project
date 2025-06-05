function sol = IK(p, L1, L2)
% twoR_inverse_kinematics:
%   Computes the inverse kinematics for a 2-link planar arm with link lengths
%   L1 and L2. The end-effector position is (x, y).
%
%   The forward kinematics for joints q1, q2 are:
%       x = L1*cos(q1) + L2*cos(q1 + q2)
%       y = L1*sin(q1) + L2*sin(q1 + q2)
%
%   The solution for q2:
%       q2 = +/- acos( (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2) )
%
%   Then q1 is found by:
%       q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2))
%
%   There can be 0, 1, or 2 feasible solutions, depending on the reachability
%   and geometry. The function returns a 2 x 2 matrix 'sol', where each row is
%   a solution [q1, q2]. If a solution is not feasible, NaN is returned.
%
% Inputs:
%   x, y  - desired end-effector coordinates in the plane
%   L1, L2 - link lengths
%
% Output:
%   sol - a 2x2 matrix of solutions, where:
%             sol(1,:) = [q1_up,   q2_up  ]  (elbow "up" or "down" depending on sign)
%             sol(2,:) = [q1_down, q2_down]
%         If no solution for that branch, the row is [NaN, NaN].

% 1) Compute r^2 = x^2 + y^2
x = p(1);
y = p(2);
r2 = x^2 + y^2;

% 2) Check for feasibility
%    The maximum reach is (L1+L2), the minimum reach is |L1-L2|.
%    If r2 > (L1+L2)^2 or r2 < (L1-L2)^2 => no real solutions.
if r2 > (L1 + L2)^2 || r2 < (L1 - L2)^2
    % Entirely unreachable
    sol = [NaN NaN; NaN NaN];
    return;
end

% 3) Compute cos(q2)
c2 = (r2 - L1^2 - L2^2) / (2*L1*L2);

% Numerical issues can make c2 slightly out of [-1, 1] range
if c2 > 1,  c2 = 1;  end
if c2 < -1, c2 = -1; end

% 4) Possible q2 solutions
q2a = acos(c2);   % "elbow" one side
q2b = -acos(c2);  % "elbow" the other side

% 5) Compute corresponding q1 for each q2
%    q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2))
%    We'll define a small helper:
q1a = atan2(y, x) - atan2(L2*sin(q2a), L1 + L2*cos(q2a));
q1b = atan2(y, x) - atan2(L2*sin(q2b), L1 + L2*cos(q2b));

% 6) Construct solutions
%    sol(1,:) => solution with q2a
%    sol(2,:) => solution with q2b
sol = [q1a, q2a;
       q1b, q2b];
end
