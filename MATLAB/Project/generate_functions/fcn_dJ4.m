function [dJ4] = fcn_dJ4(q,dq,p)

dJ4 = zeros(2,3);

  dJ4(1,1)=0;
  dJ4(1,2)=- dq(2)*(p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*...
         cos(q(2) - pi/2)) - dq(3)*p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2));
  dJ4(1,3)=- dq(2)*p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) - dq(3)*p(4)*...
         (cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2));
  dJ4(2,1)=0;
  dJ4(2,2)=- dq(2)*(p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) + p(3)*...
         sin(q(2) - pi/2)) - dq(3)*p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3)));
  dJ4(2,3)=- dq(2)*p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) - dq(3)*p(4)*...
         (cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3)));

 