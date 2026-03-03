function trajectory = genPoseC1(~)


S = load('trajectory.mat');          % loads into struct
traj = S.trajectory;

t  = traj(:,1);
x  = traj(:,2);
y  = traj(:,3);
z  = traj(:,4);
Rx = deg2rad(traj(:,5));
Ry = deg2rad(traj(:,6));
Rz = deg2rad(traj(:,7));

% plot(t,x,t,y,t,z, t,Rx,t,Ry,t,Rz);
tq = (0:1e-3:t(end)).';
trajectory = [tq, interp1(t, [x y z Rx Ry Rz], tq, 'linear')];
end