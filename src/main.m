clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu'};
mode = modes{1};
trajectory = genTrajectory(mode, 20);
trajectory(:,4) = trajectory(:,4) + 0.01;
time = trajectory(:,1);
tf = time(end);
plotTrajectory(trajectory);
trajectory = window(trajectory, 4.0, fs);
jointAngles = sp.move(trajectory);
plotMotorAngles(jointAngles);
% mData = jointAngles(:, 2:7);
% writematrix(mData, "tCSV/imuTraj.csv")
simOut = sim("SPPD.slx");
plotSimResults(simOut, trajectory, sp);
