function [Ge] = fcn_Ge(q,p)

Ge = zeros(3,1);

  Ge(1,1)=p(1)*(p(5) + p(6) + p(7));
  Ge(2,1)=p(1)*(p(7)*p(29)*cos(q(2) + q(3)) + p(7)*p(28)*sin(q(2) + q(3)) + p(6)*p(27)*cos(q(2)) +...
          p(7)*p(3)*sin(q(2)) + p(6)*p(26)*sin(q(2)));
  Ge(3,1)=p(7)*p(1)*(p(29)*cos(q(2) + q(3)) + p(28)*sin(q(2) + q(3)));

 