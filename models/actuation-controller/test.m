%% ============================================================
%  Author: Vishnu Duriseti
%  Comparison: Fixed PID vs Fuzzy-supervised PID
% ============================================================
clear; clc; close all;

%% ============================================================
%  USER INPUTS
% ========================== in ==================================
Fs_in  = 100;          % [Hz]
Tr_goal = 0.5;         % rise time
Ts_goal = 1.2;         % settling time
zeta    = 1;           % damping ratio
tau_w   = 0.20;        % actuator time constant
K_act   = 220;         % [deg/s per unit]
beta    = 0.5;         % derivative weight
sigma_w_meas = 20.0;   % [deg/s] 1-sigma measurement noise
wheel_model_uncertainty = 0.15; % [%] modeling uncertainty of wheel dynamics

%% ============================================================
%  GEOMETRY
% ============================================================
C = struct();
C.MOI_rover_zz = 18.0;
C.MOI_w1_yy=0.28; C.MOI_w2_yy=0.28; C.MOI_w3_yy=0.28; C.MOI_w4_yy=0.28;
C.d_wheel1_cg=0.35; C.d_wheel2_cg=0.35; C.d_wheel3_cg=0.35; C.d_wheel4_cg=0.35;
C.radius_w1=0.15; C.radius_w2=0.15; C.radius_w3=0.15; C.radius_w4=0.15;
C.mass_rover=52; C.mass_w1=2.6; C.mass_w2=2.6; C.mass_w3=2.6; C.mass_w4=2.6;

%% ============================================================
%  PID BASE GAINS
% ============================================================
Ts = 1/Fs_in;
omega_n = min(2.2/Tr_goal, 4/Ts_goal);
alpha   = tau_w * (1 + beta);

Kd = max((alpha - tau_w) / K_act, 0);
Kp = (alpha*(2*zeta*omega_n) - 1) / K_act;
Ki = (alpha * omega_n^2) / K_act;

fprintf('\nBase PID Gains: Kp=%.4f  Ki=%.4f  Kd=%.4f\n', Kp, Ki, Kd);

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
% starts ~0.3 Hz and sweeps to ~2 Hz by t=10s

%% --- (5) PRBS yaw with mild speed steps (system ID / robustness)
% v_knots   = [0 1.0; 3 1.4; 6 1.1; 8 1.3; 10 1.3];
% v_goal_fun   = profile_from_knots(v_knots, tau_goal);
% psi_goal_fun = @(t) 10 * prbs_like(t, 0.8);  % ~±10 deg/s, bit period ~0.8s


%% ============================================================
%  SIMULATION PARAMETERS
% ============================================================
T_end = 10;
t = 0:Ts:T_end;
N = numel(t);
a = exp(-Ts/tau_w);
b = K_act * (1 - a);
rng(6969);
w_hat_hist = zeros(N,4);
P_w_hist   = zeros(N,4);
w_hat_hist_fuzzy = zeros(N,4);
P_w_hist_fuzzy   = zeros(N,4);

%% ============================================================
%  RUN SIMULATION FUNCTION
% ============================================================

% --- Kalman filter params ---
Q = (K_act*wheel_model_uncertainty)^2;  % process noise
R = sigma_w_meas^2;                     % measurement noise
w_hat = zeros(1,4);       % KF state for each wheel
P_w   = 1e4 * ones(1,4);  % KF covariance init


[v_pid, psi_pid, w_hat_hist, P_w_hist]   = run_rover(C, t, a, b, Kp, Ki, Kd, sigma_w_meas, v_goal_fun, psi_goal_fun, Q, R, w_hat, P_w, false);
[v_fuzzy, psi_fuzzy, w_hat_hist_fuzzy, P_w_hist_fuzzy] = run_rover(C, t, a, b, Kp, Ki, Kd, sigma_w_meas, v_goal_fun, psi_goal_fun, Q, R, w_hat, P_w, true);

%% ============================================================
%  PLOTTING (Theme-Neutral)
% ============================================================

figure('Position',[80 80 1100 500]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% --- Velocity ---
nexttile;
plot(t,arrayfun(v_goal_fun,t),'--','LineWidth',1.2); hold on;
plot(t,v_pid,'LineWidth',1.4);
plot(t,v_fuzzy,'LineWidth',1.4,'Color','b');
legend('goal','PID','Fuzzy');
title('Body Velocity');
xlabel('t [s]'); ylabel('v [m/s]');
grid on;

% --- Yaw rate ---
nexttile;
plot(t,arrayfun(psi_goal_fun,t),'--','LineWidth',1.2); hold on;
plot(t,psi_pid,'LineWidth',1.4);
plot(t,psi_fuzzy,'LineWidth',1.4,'Color','b');
legend('goal','PID','Fuzzy');
title('Yaw-rate');
xlabel('t [s]'); ylabel('\psi̇ [deg/s]');
grid on;

figure(); hold on;
for i=1:4
    plot(P_w_hist(:,i))
    plot(P_w_hist_fuzzy(:,i))
end
grid minor



%% ============================================================
%  SUPPORT FUNCTIONS
% ============================================================
function [v_true, psi_true, w_hat_hist, P_w_hist] = run_rover( ...
    C, t, a, b, Kp, Ki, Kd, sigma_w_meas, v_goal_fun, psi_goal_fun, ...
    Q, R, w_hat, P_w, use_fuzzy)

    Ts = t(2)-t(1);
    w_true = zeros(1,4);
    ei = zeros(1,4);
    u  = zeros(1,4);       % IMPORTANT: KF needs previous input!
    N = numel(t);

    v_true  = zeros(N,1);
    psi_true = zeros(N,1);

    for k = 1:N
        tk = t(k);

        % goals
        v_goal   = v_goal_fun(tk);
        psi_goal = psi_goal_fun(tk);

        % true rover state
        state_true = struct('speed',0,'phi_dot',0, ...
            'w1',w_true(1),'w2',w_true(2),'w3',w_true(3),'w4',w_true(4));

        % IK: desired wheel speeds
        wg = simple_plant_IK(v_goal, psi_goal, state_true, C, Ts);

        % noisy measurement
        w_meas = w_true + sigma_w_meas * randn(1,4);

        % ==========================
        %   KALMAN FILTER PER WHEEL
        % ==========================
        for i = 1:4
            [w_hat(i), P_w(i)] = wheel_kf_step( ...
                w_hat(i), P_w(i), u(i), w_meas(i), a, b, Q, R );
        end

        % store KF state for plotting
        w_hat_hist(k,:) = w_hat;
        P_w_hist(k,:)   = P_w;

        % ==========================
        %   FILTERED ERROR
        % ==========================
        e  = wg(:)' - w_hat;  
        de = zeros(1,4);      % you can add a KF-based derivative later

        % fuzzy logic on filtered body motion
        if use_fuzzy
            [v_now, psi_now] = model_plant(w_hat', state_true, C, Ts);
            v_err   = v_goal - v_now;
            psi_err = psi_goal - psi_now;
            K_eff = fuzzy_gain_supervisor(v_err, psi_err);

            Kp_eff = Kp * K_eff.Kp;
            Ki_eff = Ki * K_eff.Ki;
            Kd_eff = Kd * K_eff.Kd;
        else
            Kp_eff = Kp; Ki_eff = Ki; Kd_eff = Kd;
        end

        % ==========================
        %   CONTROLLER OUTPUT
        % ==========================
        u = Kp_eff*e + Ki_eff*ei + Kd_eff*de;
        % u = Kp*e + Ki*ei + Kd*de;

        % ==========================
        %   ACTUATOR + INTEGRATORS
        % ==========================
        w_next = a*w_true + b*u;
        ei_next = ei + e*Ts;

        w_true = w_next;
        ei = ei_next;

        % true plant output (for sim only)
        [v_true(k), psi_true(k)] = model_plant(w_true', state_true, C, Ts);
    end
end


function K = fuzzy_gain_supervisor(v_err, psi_err)
% Fuzzy supervisor with internal LP/HP separation and rate-limited multipliers.
% Inputs are body-level errors; no other code changes required.

    % --- persistent filters (no external changes needed)
    persistent v_lp psi_lp Kprev
    if isempty(v_lp), v_lp = 0; end
    if isempty(psi_lp), psi_lp = 0; end
    if isempty(Kprev), Kprev = [1 1 1]; end

    % low-pass fraction (smaller = slower, more "steady-state" focus)
    alpha = 0.06;                 % ~ first-order LP pole

    % update LP and compute HP components
    v_lp   = (1-alpha)*v_lp   + alpha*v_err;
    psi_lp = (1-alpha)*psi_lp + alpha*psi_err;
    v_hp   = v_err  - v_lp;
    psi_hp = psi_err - psi_lp;

    % --- normalize to typical scales
    Ev_lp   = abs(v_lp)   / 1.5;
    Epsi_lp = abs(psi_lp) / 12.0;
    Ev_hp   = abs(v_hp)   / 1.5;
    Epsi_hp = abs(psi_hp) / 12.0;

    L = Ev_lp + 0.6*Epsi_lp;      % low-frequency "bias" energy
    H = Ev_hp + 0.8*Epsi_hp;      % high-frequency/edge energy

    % soft saturation (gentler than tanh)
    softsat = @(x) x ./ (1 + abs(x));

    % --- gain shapes ---
    % P: small boost on fast edges + some with bias (kept modest)
    boostP = 1.0 + 0.30 * (1 - exp(-2.2*(0.7*H + 0.3*L)));

    % I: strongest for small, persistent bias; backs off for zero/large
    % peak ~ L≈0.12 (normalized), width ≈ 0.18
    boostI = 1.0 + 0.75 * exp(-((L - 0.12)/0.18).^2);

    % D: damping for fast content only; avoid crushing I
    boostD = 1.0 + 0.35 * (1 - exp(-3.0*H));

    % --- clamps (tight → calmer wheels)
    boostP = min(max(boostP, 0.9), 1.30);
    boostI = min(max(boostI, 0.6), 1.50);
    boostD = min(max(boostD, 0.85), 1.40);

    % --- rate-limit changes to avoid thrash
    Kraw = [boostP boostI boostD];
    r    = 0.02;                                  % ≤2% step per sample
    Knew = Kprev + max(min(Kraw - Kprev, r), -r); % elementwise
    Kprev = Knew;

    K = struct('Kp', Knew(1), 'Ki', Knew(2), 'Kd', Knew(3));
end



function style_dark(ax)
    ax.Color=[0 0 0]; ax.XColor=[1 1 1]; ax.YColor=[1 1 1];
    grid(ax,'on'); ax.GridColor=[.5 .5 .5];
end

%% helpers for different input cmds

function f = profile_from_knots(knots, tau)
% knots: [t_k, y_k] with t_k strictly increasing
% Smooth C1 profile: y(t) = y0 + sum_k Δy_k * sigma((t - t_k)/tau)
    t_k = knots(:,1);
    y_k = knots(:,2);
    dy  = [y_k(1); diff(y_k)];   % increments applied at each knot
    f = @(t) y_k(1) + sum( dy.' .* sigma((t - t_k.')./tau), 2 );
end

function s = sigma(x)
% Smooth step ~ logistic; 0 → 1 with C1 continuity
    s = 0.5*(1 + tanh(x));
end

function y = sawtooth_wave(t, period, duty)
% Smooth-ish sawtooth used for slalom shaping
    if nargin<3, duty = 0.5; end
    % base saw (−1..1), then soft-limit with tanh for smooth corners
    x = 2*mod(t/period,1)-1;        % −1..1
    y = tanh( (x - (1-2*duty)) / 0.15 );
end

function f = chirp_f(t, f0, f1, T_end)
% Linear freq sweep f(t): f0→f1 over [0, T_end]
    f = f0 + (f1 - f0) .* min(max(t./T_end,0),1);
end

function u = prbs_like(t, bitT)
% Simple PRBS-like ±1 signal with smoothed edges
    if nargin<2, bitT = 1.0; end
    idx = floor(t/bitT);
    rng(12345);                     % deterministic sequence
    persistent bits; persistent lastN
    N = max(idx)+5;
    if isempty(bits) || N>lastN
        bits = 2*(rand(1,N)>0.5)-1; % ±1
        lastN = N;
    end
    raw = bits(idx+1);
    % Smooth edges by blending consecutive bits
    phase = (t - idx*bitT)/bitT;   % 0..1 within bit
    edge  = 0.15;                  % edge smoothing fraction
    s = 0.5*(1+tanh((phase-0.5)/edge));
    prev = bits(max(idx,0)+1);     % previous bit (hold for idx=0)
    u = (1-s).*prev + s.*raw;
end

function [x_hat, P] = wheel_kf_step(x_hat, P, u, z, a, b, Q, R)
    % Predict
    x_pred = a*x_hat + b*u;
    P_pred = a^2 * P + Q;

    % Innovation
    r = z - x_pred;
    S = P_pred + R;

    % Kalman gain (scalar)
    K = P_pred / S;

    % Update
    x_hat = x_pred + K * r;
    P     = (1 - K) * P_pred;
end