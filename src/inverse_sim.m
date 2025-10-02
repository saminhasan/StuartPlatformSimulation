clc; close all; clear all; %#ok<CLALL>
params;
load('cut_axes_data.mat');
Out(:,2:end) = Out(:,2:end) - mean(Out(:,2:end),1);
Out(:,[2 4 6]) = -Out(:,[2 4 6]);
data = Out;
time = data(:,1);
time = time - time(1);
tf = time(end);

plotMotorAngles(data);
jointAngles = data;
sp = StuartPlatform(r, n, rB, dB, rP, dP);

simOut = sim("SP.slx");
plotSimResults(simOut, trajectory, sp);