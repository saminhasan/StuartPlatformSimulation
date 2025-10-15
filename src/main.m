clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu'};
mode = modes{3};
trajectory = genTrajectory(mode, 30);
% trajectory(:,4) = trajectory(:,4) + 0.01;
trajectory = window(trajectory, 1.0, fs);
plotTrajectory(trajectory);

time = trajectory(:,1);
tf = time(end);
jointAngles = sp.move(trajectory);
plotMotorAngles(jointAngles);
% mData = jointAngles(:, 2:7);
% writematrix(mData, "synthTrajT10.csv")
simOut = sim("SPPD.slx");
plotSimResults(simOut, trajectory, sp);
