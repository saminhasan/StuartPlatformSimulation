clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu'};
mode = modes{2};
trajectory = genTrajectory(mode, 5);
L = sqrt(trajectory(:,2).^2 + trajectory(:,3).^2 + (trajectory(:,4)).^2);
time = trajectory(:,1);
tf = time(end);
% plotTrajectory(trajectory);
jointAngles = sp.move(trajectory);
% plotMotorAngles(jointAngles);
% simOut = sim("SPL.slx");
% plotSimResults(simOut, trajectory, sp);

