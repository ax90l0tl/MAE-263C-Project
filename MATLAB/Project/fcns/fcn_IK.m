function [q_IK] = fcn_IK(pos,p)

q_IK = zeros(2,1);

  q_IK(1,1)=pi/2 - atan2(p(4)*(1 - (abs(pos(1))^2 + abs(pos(2))^2 - p(3)^2 - p(4)^2)^2/(4*p(3)^2*...
         p(4)^2))^(1/2), p(3) + (abs(pos(1))^2 + abs(pos(2))^2 - p(3)^2 - p(4)^2)/(2*p(3))) + atan2(pos(2), pos(1));
  q_IK(2,1)=acos((pos(1)^2 + pos(2)^2 - p(3)^2 - p(4)^2)/(2*p(3)*p(4)));

 