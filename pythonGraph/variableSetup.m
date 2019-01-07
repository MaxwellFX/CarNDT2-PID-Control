clear all;
close all;
clc;

filename_P = 'P_only.csv';
filename_PI = 'PI.csv';
filename_PD = 'PD.csv';
filename_PID = 'PID.csv';
P = csvread(filename_P, 1, 0);
PI = csvread(filename_PI, 1, 0);
PD = csvread(filename_PD, 1, 0);
PID = csvread(filename_PID, 1, 0);

fileSize = 800;

x = 1:fileSize;
x = x.';

P_CTE = zeros(fileSize, 1);
P_Steer = zeros(fileSize, 1);
PI_CTE = zeros(fileSize, 1);
PI_Steer = zeros(fileSize,1);
PD_CTE = zeros(fileSize,1);
PD_Steer = zeros(fileSize,1);
PID_CTE = zeros(fileSize,1);
PID_Steer = zeros(fileSize,1);


for i = 1:fileSize
    P_CTE(i) = P(i,1);
    P_Steer(i) = P(i,2);
    PI_CTE(i) = PI(i,1);
    PI_Steer(i) = PI(i,2);
    PD_CTE(i) = PD(i,1);
    PD_Steer(i) = PD(i,2);
    PID_CTE(i) = PID(i,1);
    PID_Steer(i) = PID(i,2);
end

figure(1);
set(gcf, 'Position', [100, 100, 800, 600]);
hold on;
plot(x, P_CTE, '--','Color',[0.7, 0.5, 0], 'LineWidth', 2);
title('Cross Track Error for Different Controllers');
plot(x, PI_CTE, '--', 'Color',[1, 0, 0], 'LineWidth', 2);
plot(x, PD_CTE, '-', 'Color',[0, 1, 0], 'LineWidth', 2);
plot(x, PID_CTE, '-', 'Color',[0, 0, 1], 'LineWidth', 2)
ylim([-7, 7]);
xlabel('Steps');
ylabel('CTE');
[hleg1, hobj1] = legend('P only', 'PI only', 'PD only', 'PID', 'Location', 'southwest');
hold off;

figure(2);
set(gcf, 'Position', [100, 100, 800, 600]);
hold on;
plot(x, P_Steer, '--','Color',[0.7, 0.5, 0], 'LineWidth', 2);
title('Steering angle for Different Controllers');
plot(x, PI_Steer, '--', 'Color',[1, 0, 0], 'LineWidth', 2);
plot(x, PD_Steer, '-', 'Color',[0, 1, 0], 'LineWidth', 2);
plot(x, PID_Steer, '-', 'Color',[0, 0, 1], 'LineWidth', 2)
% ylim([-1, 1]);
xlabel('Steps');
ylabel('Steering angle');
[hleg1, hobj1] = legend('P only', 'PI only', 'PD only', 'PID', 'Location', 'southwest');
hold off;
