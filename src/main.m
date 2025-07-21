clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu'};
mode = modes{4};
trajectory = genTrajectory(mode, 5);
time = trajectory(:,1);
tf = time(end);
plotTrajectory(trajectory);
jointAngles = sp.move(trajectory);
plotMotorAngles(jointAngles);
simOut = sim("SP.slx");
plotSimResults(simOut, trajectory, sp);

