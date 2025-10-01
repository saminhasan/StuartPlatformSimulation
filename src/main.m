clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu'};
mode = modes{4};
trajectory = genTrajectory(mode, 10);
time = trajectory(:,1);
tf = time(end);
% plotTrajectory(trajectory);
jointAngles = sp.move(trajectory);
% plotMotorAngles(jointAngles);
% mData = jointAngles(:, 2:7);
 % writematrix(mData, "imuTraj.csv")
simOut = sim("SPPD.slx");
plotSimResults(simOut, trajectory, sp);

