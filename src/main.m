clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu', 'mix','c1'};
mode = modes{5};
trajectoryA = genTrajectory(mode, 20);
trajectoryA(:,6) = trajectoryA(:,6) - mean(trajectoryA(:,6)) - deg2rad(5);
plotTrajectory(trajectoryA);
rAB_body = [0.0;0;-0.333];
trajectoryB = rigid_transform(trajectoryA,rAB_body);
trajectoryB = window(trajectoryB, 4.0, fs);
time = trajectoryB(:,1);
tf = time(end);
plotTrajectory(trajectoryB);
jointAngles = sp.move(trajectoryB);
% apply reverse transform to get reference
plotMotorAngles(jointAngles);
% sp.q_rots = sp.calcQrots(trajectoryB(1,2:7), -jointAngles(1,2:7));
% % 
simOut = sim("SP.slx");
plotSimResults(simOut, trajectoryB, sp,rAB_body);
% % % % 
% mData = jointAngles(:, 2:7);
% writematrix(mData, "C1Traj.csv")
