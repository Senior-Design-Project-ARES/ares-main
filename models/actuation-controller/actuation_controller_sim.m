%% ============================================================
%  Author: Vishnu Duriseti
%  Discrete-time inner-loop wheel PID control simulation
%  with measurement noise and physical body response.
%  ------------------------------------------------------------
clear; clc; close all;

%% ============================================================
%   USER INPUTS
% ============================================================
Fs_in  = 100;          % [Hz] inner loop frequency
Tr_goal = 0.5;         % desired rise time [s]
Ts_goal = 1.2;         % desired settling time [s]
zeta    = 1;           % damping ratio
tau_w   = 0.20;        % wheel actuator time constant [s]
K_act   = 220;         % [deg/s per unit]
beta    = 0.5;         % derivative weight
sigma_w_meas = 20.0;  % [deg/s] 1-sigma wheel speed noise

%% ============================================================
%  ROVER GEOMETRY
% ============================================================
C = struct();
C.MOI_rover_zz = 18.0;
C.MOI_w1_yy=0.28; C.MOI_w2_yy=0.28; C.MOI_w3_yy=0.28; C.MOI_w4_yy=0.28;
C.d_wheel1_cg=0.35; C.d_wheel2_cg=0.35; C.d_wheel3_cg=0.35; C.d_wheel4_cg=0.35;
C.radius_w1=0.15; C.radius_w2=0.15; C.radius_w3=0.15; C.radius_w4=0.15;
C.mass_rover=52; C.mass_w1=2.6; C.mass_w2=2.6; C.mass_w3=2.6; C.mass_w4=2.6;

%% ============================================================
%  ANALYTICAL GAINS
% ============================================================
Ts = 1/Fs_in;
omega_n = min(2.2/Tr_goal, 4/Ts_goal);
alpha   = tau_w * (1 + beta);

Kd = max((alpha - tau_w) / K_act, 0);
Kp = (alpha*(2*zeta*omega_n) - 1) / K_act;
Ki = (alpha * omega_n^2) / K_act;

fprintf('\n=== Analytic Wheel PID Gains ===\n');
fprintf('  Kp = %.4f\n  Ki = %.4f\n  Kd = %.4f\n  (alpha = %.3f)\n\n', Kp, Ki, Kd, alpha);

%% ============================================================
%  GOAL PROFILES (choose ONE block to enable)
% ============================================================
% Smooth transition time constant (s) for all profiles
tau_goal = 0.12;   % smaller = snappier steps, larger = smoother ramps

% Helper: build smooth piecewise function from [t_k, y_k] knots
% v_goal_fun   = profile_from_knots([0 0;  1 1.0; 4 1.0; 4 1.6; 7 1.6; 7 0.8; 10 0.8], tau_goal); %#ok<NASGU>
% psi_goal_fun = profile_from_knots([0 0;  2 12;  5 12;  8 -8;  10 -8],               tau_goal);   %#ok<NASGU>

% ↑ This replicates your original piecewise with smooth corners.
% Comment the two lines above and uncomment ONE of the scenarios below.

%% --- (1) Mixed "city course": straights + gentle S-turns + braking
v_knots   = [0 0; 0.8 1.2; 3.2 1.2; 3.8 1.6; 6.5 1.6; 7.2 0.9; 10 0.9];
psi_knots = [0 0; 2.0  10; 3.0  10; 3.7  0; 4.6 -9; 5.6 -9; 6.4 0; 8.2 -6; 10 -6];
v_goal_fun   = profile_from_knots(v_knots,   tau_goal);
psi_goal_fun = profile_from_knots(psi_knots, tau_goal);

%% --- (2) Slalom: constant speed, alternating turns (square-wave with smoothing)
% v_goal_fun   = @(t) 1.3 + 0*t;
% psi_goal_fun = @(t) 12 * tanh(sawtooth_wave(t, 2.0, 0.5)/0.15); % ~±12 deg/s at ~0.5 Hz

%% --- (3) Tight S-turns: accelerate, left S, right S, exit straight
% v_knots   = [0 0; 1.0 1.3; 2.0 1.5; 6.0 1.5; 8.0 0.8; 10 0.8];
% psi_knots = [0 0; 2.0 12; 3.3  0; 4.6 -12; 5.9 0; 10 0];
% v_goal_fun   = profile_from_knots(v_knots,   tau_goal);
% psi_goal_fun = profile_from_knots(psi_knots, tau_goal);

%% --- (4) Yaw chirp (frequency sweep) at constant speed: great for frequency response
% v_goal_fun   = @(t) 1.2 + 0*t;
% psi_goal_fun = @(t) 12 * sin( 2*pi * chirp_f(t, 0.3, 2.0, 10) .* t );
% % starts ~0.3 Hz and sweeps to ~2 Hz by t=10s

%% --- (5) PRBS yaw with mild speed steps (system ID / robustness)
% v_knots   = [0 1.0; 3 1.4; 6 1.1; 8 1.3; 10 1.3];
% v_goal_fun   = profile_from_knots(v_knots, tau_goal);
% psi_goal_fun = @(t) 10 * prbs_like(t, 0.8);  % ~±10 deg/s, bit period ~0.8s

%% ============================================================
%  DISCRETE-TIME SIMULATION
% ============================================================
T_end = 10;
t = 0:Ts:T_end;
N = numel(t);

% First-order discrete plant coefficients
a = exp(-Ts/tau_w);
b = K_act * (1 - a);

% Initialize states
w_true = zeros(1,4);     % true wheel speeds
ei = zeros(1,4);         % integral error
rng(6969);                  % reproducible noise

% Log variables
W_true = zeros(N,4);
W_meas = zeros(N,4);
W_goal = zeros(N,4);
v_true = zeros(N,1);
psi_true = zeros(N,1);

for k = 1:N
    tk = t(k);

    % --- body-level goals
    v_goal   = v_goal_fun(tk);
    psi_goal = psi_goal_fun(tk);

    % --- IK: compute wheel goals from body goals
    state_true = struct('speed',0,'phi_dot',0, ...
        'w1',w_true(1),'w2',w_true(2),'w3',w_true(3),'w4',w_true(4));
    wg = simple_plant_IK(v_goal, psi_goal, state_true, C, Ts);
    W_goal(k,:) = wg(:)';

    % --- Measurement noise
    w_meas = w_true + sigma_w_meas * randn(1,4);
    W_meas(k,:) = w_meas;

    % % --- PID controller (measured feedback)
    e  = wg(:)' - w_meas;
    de = zeros(1,4); % could add discrete diff if desired
    u  = Kp*e + Ki*ei + Kd*de;
    
    % % --- Compute body-level fuzzy adjustment
    % [v_true_now, psi_true_now] = model_plant(w_true', state_true, C, Ts);
    % v_err  = v_goal - v_true_now;
    % psi_err = psi_goal - psi_true_now;
    % K_eff = fuzzy_gain_supervisor(v_err, psi_err);
    % 
    % % --- Apply adaptive gains globally
    % Kp_eff = Kp * K_eff.Kp;
    % Ki_eff = Ki * K_eff.Ki;
    % Kd_eff = Kd * K_eff.Kd;
    % 
    % u  = Kp_eff*e + Ki_eff*ei + Kd_eff*de;

    % --- True actuator dynamics (first-order)
    w_next = a*w_true + b*u;
    ei_next = ei + e*Ts;

    % --- Log true state
    W_true(k,:) = w_true;

    % --- Update for next iteration
    w_true = w_next;
    ei = ei_next;

    % --- Compute true body-level response
    [v_true(k), psi_true(k)] = model_plant(W_true(k,:)', state_true, C, Ts);
end

%% ============================================================
%  PLOTTING (Dark Mode)
% ============================================================
% --- Body velocity & yaw-rate (true) ---
figure('Position',[80 80 1100 520]);  
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

ax1 = nexttile;
plot(t,arrayfun(v_goal_fun,t),'--','LineWidth',1.2); hold on;
plot(t,v_true,'LineWidth',1.6);
legend('v goal','v (true)');
title('Body Velocity (True)');
xlabel('t [s]'); ylabel('v [m/s]');
grid(ax1,'on');

ax2 = nexttile;
plot(t,arrayfun(psi_goal_fun,t),'--','LineWidth',1.2); hold on;
plot(t,psi_true,'LineWidth',1.6);
legend('\psi̇ goal','\psi̇ (true)');
title('Yaw-rate (True)');
xlabel('t [s]'); ylabel('\psi̇ [deg/s]');
grid(ax2,'on');

% --- Wheel speeds (measured) ---
figure('Position',[80 80 1100 520]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

for i = 1:4
    ax = nexttile;
    plot(t,W_goal(:,i),'--','LineWidth',1.2); hold on;
    plot(t,W_meas(:,i),'LineWidth',1.0);
    legend('goal','measured');
    ylabel(sprintf('w%d [deg/s]',i));
    xlabel('t [s]');
    grid(ax,'on');
end

%% ============================================================
%  XY PATH FROM GOAL PROFILES (Perfect Tracking)
% ============================================================
T_end = 10;                 % match your goal profile horizon
t = 0:Ts:T_end;
N = numel(t);

% Evaluate goals
v = arrayfun(v_goal_fun, t);             % [m/s]
psi_dot = arrayfun(psi_goal_fun, t);     % [deg/s]
psi_dot = deg2rad(psi_dot);              % convert to rad/s

% Integrate yaw angle
psi = cumtrapz(t, psi_dot);              % heading [rad]

% Integrate position
x = cumtrapz(t, v .* cos(psi));
y = cumtrapz(t, v .* sin(psi));

% Plot trajectory with time colorbar
figure('Position',[80 80 560 500]);
scatter(x, y, 35, t, 'filled');
colormap turbo;
cb = colorbar;
cb.Label.String = 'Time [s]';
cb.Label.FontSize = 11;

xlabel('x [m]');
ylabel('y [m]');
title('XY Path (Assuming Perfect Tracking)');
axis equal;
grid on;


%% ============================================================
%  HELPER FUNCTIONS
% ============================================================

function K = fuzzy_gain_supervisor(v_err, psi_err)
    % Fuzzy-like heuristic rules (Sugeno style)
    % Inputs: body velocity error [m/s], yaw-rate error [deg/s]
    
    Ev = abs(v_err); 
    Epsi = abs(psi_err);
    
    boostP = 1 + 0.4*tanh( Ev/0.5 + Epsi/5 );
    boostI = 1 + 0.3*tanh( (0.2 - Ev) );
    boostD = 1 + 0.4*tanh( (Ev + Epsi)/2 );

    K = struct('Kp', boostP, 'Ki', boostI, 'Kd', boostD);
end

% helpers for different input cmds

function f = profile_from_knots(knots, tau)
    t_k = knots(:,1);
    y_k = knots(:,2);
    dy  = [y_k(1); diff(y_k)];
    f = @(t) y_k(1) + sum( dy.' .* sigma((t - t_k.')./tau), 2 );
end

function s = sigma(x)
    s = 0.5*(1 + tanh(x));
end

function y = sawtooth_wave(t, period, duty)
    if nargin<3, duty = 0.5; end
    x = 2*mod(t/period,1)-1;
    y = tanh( (x - (1-2*duty)) / 0.15 );
end

function f = chirp_f(t, f0, f1, T_end)
    f = f0 + (f1 - f0) .* min(max(t./T_end,0),1);
end

function u = prbs_like(t, bitT)
    if nargin<2, bitT = 1.0; end
    idx = floor(t/bitT);
    rng(12345);
    persistent bits; persistent lastN
    N = max(idx)+5;
    if isempty(bits) || N>lastN
        bits = 2*(rand(1,N)>0.5)-1;
        lastN = N;
    end
    raw = bits(idx+1);
    phase = (t - idx*bitT)/bitT;
    edge  = 0.15;
    s = 0.5*(1+tanh((phase-0.5)/edge));
    prev = bits(max(idx,0)+1);
    u = (1-s).*prev + s.*raw;
end
