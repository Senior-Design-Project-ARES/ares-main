%% Vishnu Duriseti
% Foundational setup for 4-wheel fuzzy PID inner-loop controller
clc; close all; clear;

%% Parameters
Ts = 0.01; % Sample time
sim_time = 10; % seconds

% Wheel speed references (rad/s)
omega_ref = [10; 10; 10; 10]; % Desired wheel angular velocities

% Initial measured wheel speeds
omega_meas_init = [0; 0; 0; 0];

%% Fuzzy PID initialization (conceptual)
% You'll design this fuzzy controller in Simulink using a Fuzzy Logic Controller block.
% For now, define placeholder Kp, Ki, Kd
Kp_base = 0.8;
Ki_base = 0.3;
Kd_base = 0.05;

assignin('base', 'Ts', Ts);
assignin('base', 'sim_time', sim_time);
assignin('base', 'omega_ref', omega_ref);
assignin('base', 'omega_meas_init', omega_meas_init);
assignin('base', 'Kp_base', Kp_base);
assignin('base', 'Ki_base', Ki_base);
assignin('base', 'Kd_base', Kd_base);

%% Run Simulink model
sim('four_wheel_fuzzy_pid.slx');
