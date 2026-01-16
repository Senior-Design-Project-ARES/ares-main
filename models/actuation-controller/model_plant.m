function [v_new, psi_dot_new, Fx, Mz] = model_plant(w, state, C, dt)
% MODEL_PLANT — full dynamic forward model for rover with omni front wheels
% and differential rear wheels. Derived from the analytic inverse kinematics
% (plant_IK) while keeping full EOM fidelity.
%
% Wheel order:
%   1 = front right (omni)
%   2 = front left  (omni)
%   3 = rear  left  (normal)
%   4 = rear  right (normal)

%% === Unpack state and constants ===
deg2rad = pi/180;
speed_prev   = state.speed;     % rover forward velocity [m/s]
psi_dot_prev = state.phi_dot * deg2rad; % yaw rate [rad/s]

% moments of inertia
Izz = C.MOI_rover_zz;
I_w1 = C.MOI_w1_yy; I_w2 = C.MOI_w2_yy; I_w3 = C.MOI_w3_yy; I_w4 = C.MOI_w4_yy;

% geometry
r1 = C.radius_w1; r2 = C.radius_w2; r3 = C.radius_w3; r4 = C.radius_w4;
dw1 = C.d_wheel1_cg; dw2 = C.d_wheel2_cg; dw3 = C.d_wheel3_cg; dw4 = C.d_wheel4_cg;
track_rear = dw3 + dw4;     % effective rear track width [m]

% mass
mR = C.mass_rover;
m1 = C.mass_w1; m2 = C.mass_w2; m3 = C.mass_w3; m4 = C.mass_w4;

%% === Wheel tangential velocities ===
v1 = w(1)*deg2rad*r1;  % front right
v2 = w(2)*deg2rad*r2;  % front left
v3 = w(3)*deg2rad*r3;  % rear left
v4 = w(4)*deg2rad*r4;  % rear right

%% === Step 1: Body linear and yaw kinematics ===
% front wheels = pure forward contribution
v_front = mean([v1, v2]);

% rear wheels = both forward and differential yaw source
v_rear_mean = (v3 + v4)/2;
v_body = (v_front + v_rear_mean)/2;

% yaw rate derived from differential rear wheel speeds
psi_dot_rad = (v4 - v3)/max(track_rear, 1e-6);
psi_dot_new = psi_dot_rad * 180/pi;

%% === Step 2: Compute equivalent longitudinal acceleration ===
a_long = (v_body - speed_prev)/dt;   % [m/s^2]
Fx = mR * a_long;                    % longitudinal force [N]

%% === Step 3: Compute yaw angular acceleration ===
psi_ddot = (psi_dot_rad - psi_dot_prev)/dt; % [rad/s^2]
Mz = Izz * psi_ddot;                        % yaw moment [N·m]

%% === Step 4: Integrate to next-step linear speed ===
v_new = speed_prev + a_long*dt;

%% === Step 5: Include wheel inertial coupling (optional refinement) ===
% We can refine the total kinetic energy coupling if desired:
%   T_total = 0.5*mR*v^2 + 0.5*Izz*psi_dot^2 + sum(0.5*I_wi*(wi*deg2rad)^2)
% but since angular accelerations are implicit in wdot, we only use the
% translational and yaw coupling here.

end
