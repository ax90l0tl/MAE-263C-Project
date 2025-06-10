function [phc] = fcn_phc(q,p)

phc = zeros(2,1);

  phc(1,1)=p(2) + p(4)*(cos(q(3))*cos(q(2) - pi/2) - sin(q(3))*sin(q(2) - pi/2)) + p(3)*cos(q(2) - pi/2);
  phc(2,1)=q(1) + p(4)*(cos(q(3))*sin(q(2) - pi/2) + cos(q(2) - pi/2)*sin(q(3))) + p(3)*sin(q(2) - pi/2);

 