%plots data from rovertest.slx

clc; close all;

%translate for IC
h = height(out.position.Data);
IC = out.IC;
position = out.position.Data + [ones(h,1)*IC(1) ones(h,1)*IC(2)];

%plot results
figure()
plot(out.waypoints(:,1),out.waypoints(:,2),'LineStyle','--','Marker','o','LineWidth',1)
hold on
plot(position(:,1),position(:,2),'LineWidth',1)
scatter(position(1,1),position(2,2),'marker','x','LineWidth',1,'MarkerEdgeColor','green')
scatter(position(end,1),position(end,2),'marker','x','LineWidth',1,'MarkerEdgeColor','red')
title(['Pure Pursuit Trajectory, 60s, x_0=' num2str(IC(1)) ', y_0=' num2str(IC(2)) ', theta_0 =' num2str(IC(3))])
legend('Waypoints','Path','Start','End','location','southeast')
axis equal