clc; close all; clear all; %#ok<CLALL>
addpath('quaternion_library'); 
% rB = (((609.6 ) * sqrt(3) / 6) - 59.85) /1000
[r, n, rB, dB, rP, dP] = deal(0.1, 3.5878,0.1161, 0.106/2, 0.0716025403784439,  0.02);

sp = StuartPlatform(r, n, rB, dB, rP, dP);
mode = 'imu';
[trajectory, tf, ts] = genTrajectory(mode, 5);
motorAngles = zeros(size(trajectory));
for idx = 1:size(trajectory,1)
    motorAngles(idx,:) = [trajectory(idx,1), sp.move(trajectory(idx,2:7))'];
end
plotMotorAngles(motorAngles);
plotTrajectory(trajectory);
params;
% out = sim("SP.slx");
% torque_calc(out, N, rated_torque, peak_torque);
% motion_comp(out, trajectory, sp.homez);