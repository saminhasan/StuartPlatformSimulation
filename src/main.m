clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu', 'mix'};
mode = modes{5};
trajectory = genTrajectory(mode, 10);


time = trajectory(:,1);
tf = time(end);
trajectory = window(trajectory, 2.0, fs);
plotTrajectory(trajectory);
jointAngles = sp.move(trajectory);
plotMotorAngles(jointAngles);

simOut = sim("SP.slx");
plotSimResults(simOut, trajectory, sp);
% % 
% % mData = jointAngles(:, 2:7);
% % writematrix(mData, "imuTraj.csv")
