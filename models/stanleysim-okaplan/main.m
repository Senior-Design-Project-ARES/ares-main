%plots data from stanleypathtracking.slx

clear; clc; close all;

%run the simulation
out = sim("stanleypathtracking.slx");

%get info from simout
IC = out.IC;
position = out.position.Data;% + [ones(h,1)*IC(1) ones(h,1)*IC(2)];
RefPoses = out.RefPoses;

%plot results
figure()

%plot desired trajectory
scatter(out.waypoints(:,1),out.waypoints(:,2),'Marker','o')
hold on
plot(RefPoses(1:100,1),RefPoses(1:100,2));

%plot position
plot(position(:,1),position(:,2),'LineWidth',1)
scatter(position(1,1),position(2,2),'marker','x','LineWidth',1,'MarkerEdgeColor','green')
scatter(position(end,1),position(end,2),'marker','x','LineWidth',1,'MarkerEdgeColor','red')

%make pretty
title(['Stanley Trajectory, 60s, x_0=' num2str(IC(1)) ', y_0=' num2str(IC(2)) ', \theta_0 =' num2str(IC(3))])
legend('Waypoints','Interpolated Path', 'Trajectory', 'Start','End','location','southeast')
axis equal

%desired velocity plot
figure()
plot(out.RefVelocities)
xlabel("Index")
ylabel("reference velocity (m/s)")
title("reference velocities vs index")