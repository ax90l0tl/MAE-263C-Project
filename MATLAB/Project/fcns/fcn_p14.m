function [p14] = fcn_p14(q,p)

p14 = zeros(2,1);

  p14(1,1)=p(2) + p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*cos(q(2) - pi/2);
  p14(2,1)=p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) + p(3)*sin(q(2) - pi/2);

 