clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu', 'mix','c1'};
mode = modes{1};
% trajectoryA = genTrajectory(mode, 10);
trajectoryA = genPoseSynthetic(25);

plotTrajectory(trajectoryA);
time = trajectoryA(:,1);
tf = time(end);
trajectoryA = window(trajectoryA, 2.0, fs);
jointAngles = sp.move(trajectoryA);
plotMotorAngles(jointAngles);
mData = jointAngles(:, 2:7);
writematrix(mData, "zSynthTraj.csv")
