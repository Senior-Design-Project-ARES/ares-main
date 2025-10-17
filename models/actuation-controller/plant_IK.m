function [wheel_speed] = plant_IK(speed_new,phi_dot_new,state,constants,timestep)
%{
Objective: This function calculates the needed wheel angular speed based on
the desired rover speed and rover anglular speed.

Inputs: 
    -speed: the desired speed of the rover
    -phi_dot: the desired angular rotation of the rover
    -state: the current state values of the rover (current speed and current
    angular velocity)
    -constants: the constant values of the rover (wheel radius, moment arm
    of wheels, moment of inertia of wheels, mass of wheel, coefficient of
    -friction, mass of rover)
    timestep: the selected timestep between current and new values

Output(s):
    -wheel_speed: a 4x1 array that contains the needed angular velocities
    of each wheel
%}

%% Constant Values
%state values
speed_current = state.speed; % the current wheel speeds [m/s]
phi_dot_current = state.phi_dot; % the current rover angular velocity [deg/s]
w1_current = state.w1; % the current angular velocity of the front right wheel [deg/s]
w2_current = state.w2; % the current angular velocity of the back right wheel [deg/s]
w3_current = state.w3; % the current angular velocity of the back left wheel [deg/s]
w4_current = state.w4; % the current angular velocity of the front left wheel [deg/s]

%Moments of Inertia
MOI_rover = constants.MOI_rover_zz; % the moment of inertia of the rover about the vertical axis [kg*m^2]
MOI_w1 = constants.MOI_w1_yy; % the moment of inertia of the front right wheel [kg*m^2]
MOI_w2 = constants.MOI_w2_yy; % the moment of inertia of the back right wheel [kg*m^2]
MOI_w3 = constants.MOI_w3_yy; % the moment of inertia of the back left wheel [kg*m^2]
MOI_w4 = constants.MOI_w4_yy; % the momment of inertia of the front left wheel [kg*m^2]

% distances between wheel and center of gravity
dw_1 = constants.d_wheel1_cg; % the distance between front right wheel and center of gravity [m]
dw_2 = constants.d_wheel2_cg; % the distance between back right wheel and center of gravity [m]
dw_3 = constants.d_wheel3_cg; % the distance between back left wheel and center of gravity [m]
dw_4 = constants.d_wheel4_cg; % the distance between front left wheel and center of gravity [m]

%radiuses of the wheel
radius_w1 = constants.radius_w1; % the radius of the front right wheel [m]
radius_w2 = constants.radius_w2; % the radius of the back right wheel [m]
radius_w3 = constants.radius_w3; % the radius of the back left wheel [m]
radius_w4 = constants.radius_w4; % the radius of the front left wheel [m]

%masses
mass_rover = constants.mass_rover; %the mass of the rover [kg]
mass_w1 = constants.mass_w1; % the mass of the front right wheel [kg]
mass_w2 = constants.mass_w2; % the mass of the back right wheel [kg]
mass_w3 = constants.mass_w3; % the mass of the back left wheel [kg]
mass_w4 = constants.mass_w4; % the mass of the front left wheel [kg]


%% Forces

%rover forces and moments
Fx = mass_rover*(speed_new - speed_current)/timestep; %the body fixed x-force
Mz = MOI_rover*(phi_dot_new - phi_dot_current)/timestep; %the body fixed moment about the vertical axis

%frictional forces
Ffr_left = (Fx*(dw_1-dw_2)-2*Mz) / (2*(dw_1+dw_2+dw_3+dw_4)); %the left side frictional force
Ffr_right = (Mz + Ffr_left*(dw_3_dw_4)) / (dw_1 +dw_2); %the right side frictional force

%Moments of wheels (about axel)
M_w1 = Ffr_right*radius_w1*MOI_w1/mass_w1;
M_w2 = Ffr_right*radius_w2*MOI_w2/mass_w2;
M_w3 = Ffr_left*radius_w3*MOI_w3/mass_w3;
M_w4 = Ffr_left*radius_w4*MOI_w4/mass_w4;

%angualar wheel speeds
w1_new = M_w1*timestep/MOI_w1 + w1_current;
w2_new = M_w2*timestep/MOI_w2 + w2_current;
w3_new = M_w3*timestep/MOI_w3 + w3_current;
w4_new = M_w4*timestep/MOI_w4 + w4_current;

% output
wheel_speed = [w1_new,w2_new,w3_new,w4_new];
end

