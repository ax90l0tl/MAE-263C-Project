function animateRobot(tin,Xin,p)

%------------------------------------------------------------------------
%ANIMATION
dtA = 0.05;             %Time step of animation
timeA = 0:dtA:tin(end);   %Time vector for animation with constant stepping

record = 0;             %Set to 1 if would like to record video and 0 if not
if (record)
    v = VideoWriter('CRS_Robot.avi');
    open(v)
end
f = figure;
set(f, 'doublebuffer', 'on');
for k = 1:length(timeA) 
    clf
    %Current robot state interpolation
    theta1 = interp1(tin,Xin(1,:),timeA(k));
    theta2 = interp1(tin,Xin(2,:),timeA(k));
    theta3 = interp1(tin,Xin(3,:),timeA(k));
    plotRobot([theta1; theta2; theta3],p);
    
    hold off
    F = getframe(f);
    drawnow; 
    
    if(record)
        writeVideo(v,F)
    end
end
%Closing file
if(record)
    close(v)
end

end







