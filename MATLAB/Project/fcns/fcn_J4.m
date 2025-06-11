function [J4] = fcn_J4(q,p)

J4 = zeros(2,3);

  J4(1,1)=0;
  J4(1,2)=- p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) - p(3)*sin(q(2) - pi/2);
  J4(1,3)=-p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3)));
  J4(2,1)=1;
  J4(2,2)=p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*cos(q(2) - pi/2);
  J4(2,3)=p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2));

 