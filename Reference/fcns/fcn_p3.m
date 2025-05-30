function [p3] = fcn_p3(q,p)

p3 = zeros(3,1);

  p3(1,1)=p(3)*cos(q(1))*cos(q(2));
  p3(2,1)=p(3)*cos(q(2))*sin(q(1));
  p3(3,1)=p(2) - p(3)*sin(q(2));

 