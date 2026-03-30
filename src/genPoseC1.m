function trajectory = genPoseC1(~)

S = load('trajectory.mat');
traj = S.trajectory;

t  = traj(:,1);
t  = t - t(1);
x  = traj(:,2);
y  = traj(:,3);
z  = traj(:,4);
Rx = deg2rad(traj(:,5));
Ry = deg2rad(traj(:,6));
Rz = deg2rad(traj(:,7));

tq = (0:1e-3:t(end)).';

x  = spline(t, x,  tq);
y  = spline(t, y,  tq);
z  = spline(t, z,  tq);
Rx = spline(t, Rx, tq);
Ry = spline(t, Ry, tq);
Rz = spline(t, Rz, tq);

trajectory = [tq, x, y, z, Rx, Ry, Rz];

trajectory(:,2:end) = LPFilter(trajectory(:,2:end), 1000, 20);

end

