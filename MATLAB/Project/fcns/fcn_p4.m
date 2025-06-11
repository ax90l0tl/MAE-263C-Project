function [p4] = fcn_p4(q,p)

p4 = zeros(2,1);

  p4(1,1)=p(2) + p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*cos(q(2) - pi/2);
  p4(2,1)=q(1) + p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) + p(3)*sin(q(2) - pi/2);

 