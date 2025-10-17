function trajectory = genPoseCam(n)
    dt = 1e-3;
    % data = readtable('CSV/Rots_raw.csv', 'CommentStyle', '#');
    data = table2array(readtable('CSV/Rots100.csv','CommentStyle','#'));
    ts = mean(diff(data(:,1)));
    tEnd = (data(end,1)+ts)*n - ts;
    te = (0:dt:tEnd)';
    rawTrajectory = zeros(height(data)*n,7);
    rawTrajectory(:,1) = (0:ts:tEnd)';
    rawTrajectory(:,4:7) = repmat(data(:,4:7) - mean(data(:,4:7),1), n, 1);
    rawTrajectory(:,5:7) = deg2rad(rawTrajectory(:,5:7));
    rawTrajectory(:,6) = rawTrajectory(:,6) - min(rawTrajectory(:,6));
    trajectory = [te, spline(rawTrajectory(:,1), rawTrajectory(:,2:7)', te')'];
end