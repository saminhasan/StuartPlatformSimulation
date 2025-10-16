function trajectory = genPoseCam(n)
    dt = 1e-3;
    % data = readtable('CSV/Rots_raw.csv', 'CommentStyle', '#');
    data = readtable('CSV/Rots100.csv', 'CommentStyle', '#');
    ts = mean(diff(data.time));
    t = (0:ts:(data.time(end) + ts) * n - ts)';
    % Replicate and process signals
    te = (0:dt:t(end))';

    x = zeros(size(t));
    y = zeros(size(t));
    z = repmat(data.Z - mean(data.Z), n, 1);
    Rx = repmat(deg2rad(data.X_rot - mean(data.X_rot)), n, 1);
    Ry = repmat(deg2rad(data.Y_rot - min(data.Y_rot)), n, 1);    
    % Ry = -repmat(deg2rad(data.Y_rot), n, 1);    
    Rz = repmat(deg2rad(data.Z_rot - mean(data.Z_rot)), n, 1);
    rawTrajectory = [x, y, z, Rx, Ry, Rz];

    % Resample at 1kHz using spline
    trajectory = [te, spline(t, rawTrajectory', te)'];
end