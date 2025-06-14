function plotRobot(X,p)

params = p.params;

q = X(1:3);
% p1 = [0 0]';
p2 = fcn_p2(q,params);
p3 = fcn_p3(q,params);
p4 = fcn_p4(q,params);

chain = [p2 p3 p4];

rectangle('Position',[p2(1)-params(3)/4,p2(2)-params(3)/4, params(3)/2,params(3)/2],'FaceColor','blue', 'EdgeColor','none')
hold on
plot(chain(1,:),chain(2,:),'color','black','linewidth',4);
plot(chain(1,:),chain(2,:),'ocyan','LineWidth',4);

yline(0,'--black')
grid on
axis square

limits = (params(3)+params(4))*[-1.1 , 1.1 , -0.1, 1.1] + [ 0, 0, 0, q(1)];
% limits = [-0.05 0.05 -0.05 0.05];
axis(limits)
xlabel('x [m]','fontsize',12)
ylabel('y [m]','fontsize',12)


end
