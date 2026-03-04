% ==============================================================
%   Vishnu Duriseti
%   PA-13 3000 lb Linear Actuator: Speed vs. Load
%   Polyfit interpolation and visualization
% ==============================================================
clc; close all; clear;

% Data (approx. from manufacturer chart)
load_lbs = [0 500 1000 1500 2000 2500 3000];
speed_ips = [0.33 0.28 0.24 0.21 0.18 0.16 0.14];   % inches/second

% Fit polynomial of chosen degree (try 1, 2, or 3)
deg = 2;
p = polyfit(load_lbs, speed_ips, deg);

% Generate smooth load range for plotting
load_dense = linspace(0, 3000, 200);
speed_fit = polyval(p, load_dense);

% Plot
figure; hold on; grid minor;
plot(load_lbs, speed_ips, 'ko', 'MarkerFaceColor','y', 'DisplayName','Data Points');
plot(load_dense, speed_fit, 'r-', 'LineWidth',1.5, 'DisplayName',sprintf('Polyfit (deg=%d)', deg));
xlabel('Load (lbs)');
ylabel('Speed (in/s)');
title('PA-13 3000 lb Actuator: Speed vs. Load');
legend('show','Location','northeast');
set(gca,'FontSize',12);

% Print polynomial coefficients and example prediction
disp('Polynomial Coefficients (highest power first):');
disp(p);

% Example: predict speed at 1200 lbs
test_load = 1200;
pred_speed = polyval(p, test_load);
fprintf('Predicted speed at %.0f lbs = %.3f in/s\n', test_load, pred_speed);
