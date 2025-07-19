function [pose, tf, Ts] = genaratePoseTK(n)
    data = readtable('Rots100.csv', 'CommentStyle', '#');

    Ts = mean(diff(data.time));
    t = (0:Ts:(data.time(end) + Ts) * n - Ts)';
    
    % Replicate and process signals
    x = zeros(size(t));
    y = zeros(size(t));
    z = repmat(data.Z - mean(data.Z), n, 1);

    Ry = repmat(deg2rad(data.Y_rot - mean(data.Y_rot)), n, 1);    
    Rz = repmat(deg2rad(data.Z_rot - mean(data.Z_rot)), n, 1);
    Rx = repmat(deg2rad(data.X_rot - mean(data.X_rot)), n, 1);

    raw_pose = [t, x, y, z, Rx, Ry, Rz];
    % Resample at 1kHz using spline
    te = t(1):1e-3:t(end);
    tf = te(end);
    % data_new = interp1(t, raw_pose(:,2:end), te, 'spline');
    data_new = spline(t, raw_pose(:,2:end)', te)';
    pose = [te', data_new];
end