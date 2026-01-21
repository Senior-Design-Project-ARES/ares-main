%% ============================================================
%  Author: Vishnu Duriseti
%  Fuzzy PID Monte Carlo tuner comparison vs. base PID
% ============================================================
clear; clc; close all;

%% ============================================================
%  USER INPUTS
% ============================================================
Fs_in  = 100;          
Tr_goal = 0.5;         
Ts_goal = 1.2;         
zeta    = 1;           
tau_w   = 0.20;        
K_act   = 220;         
beta    = 0.5;         
sigma_w_meas = 10.0;   

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
%  BASE PID GAINS
% ============================================================
Ts = 1/Fs_in;
omega_n = min(2.2/Tr_goal, 4/Ts_goal);
alpha   = tau_w * (1 + beta);
Kd = max((alpha - tau_w) / K_act, 0);
Kp = (alpha*(2*zeta*omega_n) - 1) / K_act;
Ki = (alpha * omega_n^2) / K_act;
fprintf('\nBase PID Gains: Kp=%.4f  Ki=%.4f  Kd=%.4f\n', Kp, Ki, Kd);

%% ============================================================
%  GOAL PROFILES
% ============================================================
function f = chirp_f(t, f0, f1, T_end)
% Linear freq sweep f(t): f0→f1 over [0, T_end]
    f = f0 + (f1 - f0) .* min(max(t./T_end,0),1);
end

v_goal_fun   = @(t) 1.2 + 0*t;
psi_goal_fun = @(t) 12 * sin( 2*pi * chirp_f(t, 0.3, 2.0, 10) .* t );
% starts ~0.3 Hz and sweeps to ~2 Hz by t=10s

%% ============================================================
%  SIMULATION SETUP
% ============================================================
T_end = 10;
t = 0:Ts:T_end;
N = numel(t);
a = exp(-Ts/tau_w);
b = K_act * (1 - a);
rng(6969);

%% ============================================================
%  1. Run Base PID
% ============================================================
[v_pid, psi_pid] = run_rover(C, t, a, b, Kp, Ki, Kd, sigma_w_meas, v_goal_fun, psi_goal_fun, false, []);

v_goal = arrayfun(v_goal_fun,t);
psi_goal = arrayfun(psi_goal_fun,t);
IAE_v_pid   = trapz(t, abs(v_goal - v_pid));
IAE_psi_pid = trapz(t, abs(psi_goal - psi_pid));

%% ============================================================
%  2. Monte Carlo Fuzzy Tuning
% ============================================================
N_runs = 5000;
param_results = zeros(N_runs,5); % [kp_gain ki_gain kd_gain totalIAE best_idx]
v_best = []; psi_best = []; bestIAE = inf;
v_goal = arrayfun(v_goal_fun,t)';
psi_goal = arrayfun(psi_goal_fun,t)';

for i = 1:N_runs
    % Random fuzzy parameters (gain multipliers & sensitivities)
    F.kp_gain = 0.3 + 0.5*rand(); 
    F.ki_gain = 0.2 + 0.4*rand(); 
    F.kd_gain = 0.3 + 0.5*rand(); 
    F.kp_Ev   = 0.3 + 0.4*rand();
    F.kp_Epsi = 3 + 3*rand();

    [v_fuzzy, psi_fuzzy] = run_rover(C, t, a, b, Kp, Ki, Kd, sigma_w_meas, v_goal_fun, psi_goal_fun, true, F);

    % Ensure vectors are column
    v_fuzzy = v_fuzzy(:);
    psi_fuzzy = psi_fuzzy(:);

    % Compute IAE scalars robustly
    IAE_v   = trapz(t, abs(v_goal - v_fuzzy));
    IAE_psi = trapz(t, abs(psi_goal - psi_fuzzy));

    % Collapse any stray vector dimension
    IAE_v   = sum(IAE_v(:));
    IAE_psi = sum(IAE_psi(:));

    totalIAE = IAE_v + 0.5*IAE_psi;  % weighted sum (scalar)
    param_results(i,:) = [F.kp_gain F.ki_gain F.kd_gain totalIAE i];

    if totalIAE < bestIAE
        bestIAE = totalIAE;
        v_best = v_fuzzy; psi_best = psi_fuzzy; bestF = F;
    end
end

%% ============================================================
%  3. Plot Results
% ============================================================
set(0,'DefaultFigureColor',[0 0 0]);
set(0,'DefaultAxesColor',[0 0 0]);
set(0,'DefaultAxesXColor',[1 1 1]);
set(0,'DefaultAxesYColor',[1 1 1]);

% --- Response Comparison ---
figure('Color','k','Position',[80 80 1100 500]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

ax1 = nexttile;
plot(t,v_goal,'--','LineWidth',1.2,'Color',[0.7 0.7 0.7]); hold on;
plot(t,v_pid,'LineWidth',1.2,'Color',[0.2 0.7 1]);
plot(t,v_best,'LineWidth',1.2,'Color',[1 0.6 0.2]);
legend('goal','PID','Best Fuzzy','TextColor','w');
title('Body Velocity','Color','w'); xlabel('t [s]'); ylabel('v [m/s]'); style_dark(ax1);

ax2 = nexttile;
plot(t,psi_goal,'--','LineWidth',1.2,'Color',[0.7 0.7 0.7]); hold on;
plot(t,psi_pid,'LineWidth',1.2,'Color',[0.2 0.7 1]);
plot(t,psi_best,'LineWidth',1.2,'Color',[1 0.6 0.2]);
legend('goal','PID','Best Fuzzy','TextColor','w');
title('Yaw-rate','Color','w'); xlabel('t [s]'); ylabel('\psi̇ [deg/s]'); style_dark(ax2);

% --- Performance Scatter ---
figure('Color','k','Position',[100 100 500 400]);
scatter3(param_results(:,1), param_results(:,2), param_results(:,3), 80, -param_results(:,4), 'filled');
xlabel('Kp_{gain}'); ylabel('Ki_{gain}'); zlabel('Kd_{gain}');
title('Monte Carlo Fuzzy Parameter Performance','Color','w');
cb = colorbar; cb.Label.String = '-IAE (better ↑)'; cb.Color = 'w';
style_dark(gca);

%% ============================================================
%  SUPPORT FUNCTIONS
% ============================================================
function [v_true, psi_true] = run_rover(C, t, a, b, Kp, Ki, Kd, sigma_w_meas, v_goal_fun, psi_goal_fun, use_fuzzy, F)
    Ts = t(2)-t(1);
    w_true = zeros(1,4); ei = zeros(1,4);
    N = numel(t);
    v_true = zeros(N,1); psi_true = zeros(N,1);

    for k = 1:N
        tk = t(k);
        v_goal   = v_goal_fun(tk);
        psi_goal = psi_goal_fun(tk);
        state_true = struct('speed',0,'phi_dot',0, ...
            'w1',w_true(1),'w2',w_true(2),'w3',w_true(3),'w4',w_true(4));
        wg = simple_plant_IK(v_goal, psi_goal, state_true, C, Ts);
        w_meas = w_true + sigma_w_meas * randn(1,4);
        e  = wg(:)' - w_meas;
        de = zeros(1,4);

        if use_fuzzy
            [v_now, psi_now] = model_plant(w_true', state_true, C, Ts);
            v_err  = v_goal - v_now; psi_err = psi_goal - psi_now;
            K_eff = fuzzy_gain_supervisor(v_err, psi_err, F);
            Kp_eff = Kp * K_eff.Kp; Ki_eff = Ki * K_eff.Ki; Kd_eff = Kd * K_eff.Kd;
        else
            Kp_eff = Kp; Ki_eff = Ki; Kd_eff = Kd;
        end

        u = Kp_eff*e + Ki_eff*ei + Kd_eff*de;
        w_next = a*w_true + b*u;
        ei_next = ei + e*Ts;
        w_true = w_next; ei = ei_next;
        [v_true(k), psi_true(k)] = model_plant(w_true', state_true, C, Ts);
    end
end

function K = fuzzy_gain_supervisor(v_err, psi_err, F)
    Ev   = abs(v_err)/1.5;      % normalize
    Epsi = abs(psi_err)/12; 
    alpha = 0.4;                % lower slope
    boostP = 1 + F.kp_gain*tanh(alpha*(Ev/F.kp_Ev + Epsi/F.kp_Epsi));
    boostI = 1 + F.ki_gain*tanh(alpha*(0.3 - Ev));
    boostD = 1 + F.kd_gain*tanh(alpha*(Ev + Epsi));
    K = struct('Kp', boostP, 'Ki', boostI, 'Kd', boostD);
end

function style_dark(ax)
    ax.Color=[0 0 0]; ax.XColor=[1 1 1]; ax.YColor=[1 1 1];
    grid(ax,'on'); ax.GridColor=[.5 .5 .5];
end
