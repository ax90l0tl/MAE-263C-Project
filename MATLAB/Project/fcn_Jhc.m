function [Jhc] = fcn_Jhc(q,p)

Jhc = zeros(2,3);

  Jhc(1,1)=0;
  Jhc(1,2)=- p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) - p(3)*sin(q(2) - pi/2);
  Jhc(1,3)=-p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3)));
  Jhc(2,1)=1;
  Jhc(2,2)=p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*cos(q(2) - pi/2);
  Jhc(2,3)=p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2));

 