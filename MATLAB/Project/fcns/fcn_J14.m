function [J14] = fcn_J14(q,p)

J14 = zeros(2,3);

  J14(1,1)=0;
  J14(1,2)=- p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) - p(3)*sin(q(2) - pi/2);
  J14(1,3)=-p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3)));
  J14(2,1)=0;
  J14(2,2)=p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*cos(q(2) - pi/2);
  J14(2,3)=p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2));

 