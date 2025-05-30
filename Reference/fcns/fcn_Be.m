function [Be] = fcn_Be(q,p)

Be = zeros(3,3);

  Be(1,1)=1;
  Be(1,2)=0;
  Be(1,3)=0;
  Be(2,1)=0;
  Be(2,2)=1;
  Be(2,3)=0;
  Be(3,1)=0;
  Be(3,2)=0;
  Be(3,3)=1;

 