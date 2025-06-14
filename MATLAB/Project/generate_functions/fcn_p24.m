function [p24] = fcn_p24(q,p)

p24 = zeros(2,1);

  p24(1,1)=p(3) + p(4)*cos(q(3));
  p24(2,1)=p(4)*sin(q(3));

 