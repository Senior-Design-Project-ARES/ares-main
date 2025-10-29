%% ============================================================
%  Author: Vishnu Duriseti
%  Analytically derive and simulate inner loop wheel PID control
%  given body-level rise/settle specs and physical constants.
%
%  Inner loop (100 Hz): PID per wheel tracking wheel-speed goals
%
%  ------------------------------------------------------------
clear; clc;

%% ============================================================
%  USER INPUTS  🔧  (All parameters you can tune or measure)
% ============================================================

% --- Frequency / duty parameters ---
Fs_in  = 100;          % [Hz] inner (actuation) loop frequency
Fs_out = 25;           % [Hz] outer (trajectory) loop frequency

% --- Desired performance (body-level) ---
Tr_goal = 0.5;         % [s]  desired rise time (time to ~90%)
Ts_goal = 1.2;         % [s]  desired settling time (<2% error)
zeta    = 0.707;       % damping ratio (~0.7 = critically damped)

% --- Wheel actuator dynamics ---
tau_w = 0.20;          % [s] time constant (measure from step response)
                       %  → time for wheel speed to reach 63% of final
K_act = 220;           % [deg/s per unit] actuator gain
                       %  → steady-state wheel speed per unit command
                       %  → find via:  w_ss / u_ss from steady test
beta   = 0.5;          % [-] derivative weight (0=no D, >1=more damping)

% --- Low-pass filter for measured wheel speed ---
tau_f = 0.08;          % [s]  measurement lag or filter cutoff

%% ============================================================
%  ROVER GEOMETRY (for plant_IK and body_state_from_wheels)
% ============================================================
C = struct();
C.MOI_rover_zz = 18.0;
C.MOI_w1_yy=0.28; C.MOI_w2_yy=0.28; C.MOI_w3_yy=0.28; C.MOI_w4_yy=0.28;
C.d_wheel1_cg=0.35; C.d_wheel2_cg=0.35; C.d_wheel3_cg=0.35; C.d_wheel4_cg=0.35;
C.radius_w1=0.15; C.radius_w2=0.15; C.radius_w3=0.15; C.radius_w4=0.15;
C.mass_rover=52; C.mass_w1=2.6; C.mass_w2=2.6; C.mass_w3=2.6; C.mass_w4=2.6;

%% ============================================================
%  ANALYTICAL GAIN CALCULATION (from specs)
% ============================================================
Ts = 1/Fs_in;

omega_n = min(2.2/Tr_goal, 4/Ts_goal);        % natural frequency [rad/s]
alpha   = tau_w * (1 + beta);                 % effective 2nd-order scaling

Kd = max((alpha - tau_w) / K_act, 0);         % [s]
Kp = (alpha*(2*zeta*omega_n) - 1) / K_act;    % proportional gain
Ki = (alpha * omega_n^2) / K_act;             % integral gain

fprintf('\n=== Analytic Wheel PID Gains ===\n');
fprintf('  Kp = %.4f\n  Ki = %.4f\n  Kd = %.4f\n  (alpha = %.3f)\n\n', Kp, Ki, Kd, alpha);

%% ============================================================
%  SIMULATION SETUP
% ============================================================
T_end = 10;  t = 0:Ts:T_end;
v_goal   = zeros(size(t));   v_goal(t>=1 & t<4) = 1.0; v_goal(t>=4 & t<7) = 1.6; v_goal(t>=7)=0.8;
psi_goal = zeros(size(t));   psi_goal(t>=2 & t<5)=12;  psi_goal(t>=8)=-8;

state = struct('speed',0,'phi_dot',0,'w1',0,'w2',0,'w3',0,'w4',0);
w = zeros(4,1); w_f = zeros(4,1); u = zeros(4,1);
e_int = zeros(4,1); e_prev = zeros(4,1);

a_f = Ts / tau_f;  % discrete filter coeff
n = numel(t);
vlog=zeros(1,n); pslog=zeros(1,n);
wlog=zeros(4,n); wgoal=zeros(4,n); ulog=zeros(4,n);

%% ============================================================
%  SIMULATION LOOP
% ============================================================
for k = 1:n
    [v_est, psi_est] = body_state_from_wheels(w, C);
    state.speed=v_est; state.phi_dot=psi_est;
    state.w1=w(1); state.w2=w(2); state.w3=w(3); state.w4=w(4);

    wg = simple_plant_IK(v_goal(k), psi_goal(k), state, C, Ts); % wheel speed goals
    wg = wg(:);  % Force column vector (4x1)

    w_f = w_f + a_f*(w - w_f);          % filter measured wheel speeds
    e  = wg - w_f;                      % error
    de = (e - e_prev)/Ts; e_prev = e;
    e_int = e_int + e*Ts;

    u = Kp*e + Ki*e_int + Kd*de;        % PID control
    wdot = (K_act*u - w)/tau_w;         % wheel dynamics
    w = w + Ts*wdot;

    [v_est, psi_est] = body_state_from_wheels(w, C);
    vlog(k)=v_est; pslog(k)=psi_est;
    wlog(:,k)=w; wgoal(:,k)=wg; ulog(:,k)=u;
end

%% ============================================================
%  PLOTTING (Dark Mode)
% ============================================================
set(0,'DefaultFigureColor',[0 0 0]);
figure('Color','k','Position',[70 70 1100 420]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
ax1=nexttile; plot(t,v_goal,'--','LineWidth',1.2); hold on; plot(t,vlog,'LineWidth',1.6);
legend('v goal','v actual','TextColor','w'); title('Velocity','Color','w');
xlabel('t [s]','Color','w'); ylabel('v [m/s]','Color','w'); style_dark(ax1);
ax2=nexttile; plot(t,psi_goal,'--','LineWidth',1.2); hold on; plot(t,pslog,'LineWidth',1.6);
legend('\psi̇ goal','\psi̇ actual','TextColor','w'); title('Yaw-rate','Color','w');
xlabel('t [s]','Color','w'); ylabel('\psi̇ [deg/s]','Color','w'); style_dark(ax2);

figure('Color','k','Position',[80 80 1100 520]); tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
for i=1:4
    ax=nexttile;
    plot(t,wgoal(i,:),'--','LineWidth',1.2); hold on;
    plot(t,wlog(i,:) ,'LineWidth',1.6);
    legend('goal','actual','TextColor','w');
    ylabel(sprintf('w%d [deg/s]',i),'Color','w');
    if i>=3, xlabel('t [s]','Color','w'); end; style_dark(ax);
end
ax=nexttile([1 2]); plot(t,ulog(1,:),t,ulog(2,:),t,ulog(3,:),t,ulog(4,:),'LineWidth',1.0);
legend('w1','w2','w3','w4','TextColor','w'); title('Control Effort','Color','w');
xlabel('t [s]','Color','w'); ylabel('u [-]','Color','w'); style_dark(ax);

%% ============================================================
%  HELPER FUNCTIONS
% ============================================================
function [v_est, psi_dot_deg] = body_state_from_wheels(w_deg, C)
    wr = pi/180;
    v1=(w_deg(1)*wr)*C.radius_w1; v2=(w_deg(2)*wr)*C.radius_w2;
    v3=(w_deg(3)*wr)*C.radius_w3; v4=(w_deg(4)*wr)*C.radius_w4;
    v_est = (v1+v2+v3+v4)/4;
    v_right=(v1+v2)/2; v_left=(v3+v4)/2;
    track = max(C.d_wheel1_cg+C.d_wheel2_cg+C.d_wheel3_cg+C.d_wheel4_cg,1e-6);
    psi_dot_rad = (v_right - v_left)/track;
    psi_dot_deg = psi_dot_rad*180/pi;
end
function style_dark(ax)
    ax.Color=[0 0 0]; ax.XColor=[1 1 1]; ax.YColor=[1 1 1];
    grid(ax,'on'); ax.GridColor=[.5 .5 .5];
end
