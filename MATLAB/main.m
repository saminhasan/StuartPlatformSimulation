clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
mode = 'sin';
[trajectory, tf, ts] = genTrajectory(mode, 5);
plotTrajectory(trajectory);

motorAngles = zeros(size(trajectory));
for idx = 1:size(trajectory,1)
    motorAngles(idx,:) = [trajectory(idx,1), sp.move(trajectory(idx,2:7))'];
end
plotMotorAngles(motorAngles);
out = sim("SPPD.slx");
plotSimResults(out, trajectory, motorAngles, sp);