clc; close all; clear all; %#ok<CLALL>
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
modes = {'sin', 'syn', 'cam', 'imu', 'mix','c1'};
mode = modes{5};
trajectoryA = genTrajectory(mode, 20);
trajectoryA(:,6) = trajectoryA(:,6) - mean(trajectoryA(:,6)) - deg2rad(15);
rAB_body = [0.0;0;0.15];
trajectoryB = rigid_transform(trajectoryA,rAB_body);
trajectoryB = window(trajectoryB, 4.0, fs);
time = trajectoryB(:,1);
tf = time(end);
jointAngles = sp.move(trajectoryB);
% sp.q_rots = sp.calcQrots(trajectoryB(1,2:7), jointAngles(1,2:7));
% % 
simOut = sim("SP.slx");

