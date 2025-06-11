function [p3] = fcn_p3(q,p)

p3 = zeros(2,1);

  p3(1,1)=p(2) + p(3)*cos(q(2) - pi/2);
  p3(2,1)=q(1) + p(3)*sin(q(2) - pi/2);

 