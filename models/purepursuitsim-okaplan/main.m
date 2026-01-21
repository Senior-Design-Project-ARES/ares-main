%plots data from stanleypathtracking.slx

clear; clc; close all;

%set noise parameters
minNoise = -0.0000001;
maxNoise = 0.000001;

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
plot(RefPoses(1:200,1),RefPoses(1:200,2));

%plot position
plot(position(:,1),position(:,2),'LineWidth',1)
scatter(position(1,1),position(2,2),'marker','x','LineWidth',1,'MarkerEdgeColor','green')
scatter(position(end,1),position(end,2),'marker','x','LineWidth',1,'MarkerEdgeColor','red')

%find time to reach end
[~,finIndex] = min(vecnorm(position - [ones(height(position),1)*position(end,1) ones(height(position),1)*position(end,2)],2,2));
tend = out.tout(finIndex);

%make pretty
title(['Stanley Trajectory, ' num2str(tend) 's, x_0=' num2str(IC(1)) ', y_0=' num2str(IC(2)) ', \theta_0 =' num2str(IC(3))])
legend('Waypoints','Interpolated Path', 'Trajectory', 'Start','End','location','southeast')
axis equal

%desired velocity plot
figure()
plot(out.RefVelocities)
xlabel("Index")
ylabel("reference velocity (m/s)")
title("reference velocities vs index")

%crosstrack error plot
figure()
y = lowpass(out.CTError,0.003,0.1);
plot(out.tout,out.CTError)
hold on
plot(out.tout,y,'LineWidth',2)
xlabel("time (s)")
ylabel("crosstrack error (m)")
title("crosstrack error vs. time")
legend('Data','Data w/ LPF')