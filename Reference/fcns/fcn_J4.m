function [J4] = fcn_J4(q,p)

J4 = zeros(3,3);

  J4(1,1)=p(4)*(sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(2))*cos(q(3))*sin(q(1))) - p(3)*cos(q(2))*sin(q(1));
  J4(1,2)=- p(4)*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2))) - p(3)*cos(q(1))*sin(q(2));
  J4(1,3)=-p(4)*(cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2)));
  J4(2,1)=p(4)*(cos(q(1))*cos(q(2))*cos(q(3)) - cos(q(1))*sin(q(2))*sin(q(3))) + p(3)*cos(q(1))*cos(q(2));
  J4(2,2)=- p(4)*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2))) - p(3)*sin(q(1))*sin(q(2));
  J4(2,3)=-p(4)*(cos(q(2))*sin(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)));
  J4(3,1)=0;
  J4(3,2)=- p(4)*(cos(q(2))*cos(q(3)) - sin(q(2))*sin(q(3))) - p(3)*cos(q(2));
  J4(3,3)=-p(4)*(cos(q(2))*cos(q(3)) - sin(q(2))*sin(q(3)));

 