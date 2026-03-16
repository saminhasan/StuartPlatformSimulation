clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu', 'mix','c1'};
mode = modes{1};
trajectoryA = genTrajectory(mode, 10);
% trajectoryA(:,6) = trajectoryA(:,6) - deg2rad(6);
plotTrajectory(trajectoryA);

rAB_body = [0;0;-0.25];

trajectoryB = rigid_transform(trajectoryA,rAB_body);
time = trajectoryB(:,1);
tf = time(end);
trajectoryB = window(trajectoryB, 2.0, fs);
% plotTrajectory(trajectoryB);
jointAngles = sp.move(trajectoryB);
plotMotorAngles(jointAngles);
% 
simOut = sim("SP.slx");
plotSimResults(simOut, trajectoryB,trajectoryA, sp,rAB_body);
% % 
mData = jointAngles(:, 2:7);
writematrix(mData, "RyTraj.csv")
