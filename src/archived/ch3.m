n=5;
    dt = 1e-3;
    data = readtable('CSV/Rots100.csv', 'CommentStyle', '#');
    % data = readtable('CSV/Rots_raw.csv', 'CommentStyle', '#');
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
% compute world-frame vertical acceleration
accZ_world = filter([1, -2, 1], dt^2, trajectory(:,4));

roll = trajectory(:,5);
pitch = trajectory(:,6);
yaw = trajectory(:,7);

% Preallocate
accZ_body = zeros(size(accZ_world));

% Compute per-sample rotation and transform
for k = 1:length(accZ_world)
    cr = cos(roll(k)); sr = sin(roll(k));
    cp = cos(pitch(k)); sp = sin(pitch(k));
    cy = cos(yaw(k)); sy = sin(yaw(k));

    % body->world rotation
    R = [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr;
          sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr;
          -sp,    cp*sr,             cp*cr ];

    % world acceleration vector
    a_world = [0; 0; accZ_world(k)];

    % convert to body frame
    a_body = R' * a_world;

    accZ_body(k) = a_body(3);
end
    g = 9.80665;
accZ_body = accZ_body(3:end);
accZ_body = accZ_body - min(accZ_body) - g;
    fh = 0.5;
    fl = 15;
    method = 4;
    file_number = 1;
    DATA_PATH = fullfile(pwd, 'IMUData');
    filelist = string({dir(fullfile(DATA_PATH, '*.mat')).name});
    data = load(fullfile(DATA_PATH, filelist(file_number)));
    imu_data = data.data_mtl;% Data is saved in matrix name data_mtl
        imu_data = imu_data(4000:8000,:);

    n = length(imu_data);
    % plotIMUData(imu_data);
    time = imu_data(:, 1); % Time (s)
    time(:, 1) = time(:, 1) - time(1,1); % Normalize time to start at zero
    dt = mean(diff(time));
    fs = 1/ dt;


    z_acc = imu_data(:, 4);               % Z Linear Acceleration (m/s^2)
    z_acc = z_acc - min(z_acc) - g - 1.8;
% plot
figure;
hold on
plot(time , z_acc, DisplayName='$\ddot{z}$ (imu)');
plot((trajectory(3:end,1) /1.16) +0.11, accZ_body, DisplayName='$\ddot{z}$ (mocap)');
xlabel('Time (s)');
ylabel('Acceleration $[\mathrm{m/s^2}]$', 'Interpreter','latex');
    leg1 = legend('show','Location','best'); set(leg1,'Interpreter','latex');
title('Body-frame vertical acceleration comparison');
% end