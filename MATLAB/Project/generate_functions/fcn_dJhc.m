function [dJhc] = fcn_dJhc(q,dq,p)

dJhc = zeros(2,3);

  dJhc(1,1)=0;
  dJhc(1,2)=- dq(2)*(p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*...
         cos(q(2) - pi/2)) - dq(3)*p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2));
  dJhc(1,3)=- dq(2)*p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) - dq(3)*p(4)*...
         (cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2));
  dJhc(2,1)=0;
  dJhc(2,2)=- dq(2)*(p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) + p(3)*...
         sin(q(2) - pi/2)) - dq(3)*p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3)));
  dJhc(2,3)=- dq(2)*p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) - dq(3)*p(4)*...
         (cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3)));

 