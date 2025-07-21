function trajectory = genPoseImu(num_cycle) %#ok<INUSD>
    addpath(fullfile(pwd,"quaternion_library"));
    fh = 0.5;
    fl = 25;
    method = 4;
    file_number = 1;
    DATA_PATH = fullfile(pwd, 'IMUData');
    filelist = string({dir(fullfile(DATA_PATH, '*.mat')).name});
    data = load(fullfile(DATA_PATH, filelist(file_number)));
    imu_data = data.data_mtl;% Data is saved in matrix name data_mtl
    n = length(imu_data);
    % plotIMUData(imu_data);
    time = imu_data(:, 1); % Time (s)
    time(:, 1) = time(:, 1) - time(1,1); % Normalize time to start at zero
    dt = mean(diff(time));
    fs = 1/ dt;

    accel = imu_data(:,2:4);
    gyro  = deg2rad(imu_data(:,5:7));

    if (method ==1)
        fuse = imufilter('SampleRate', fs);
        q1 = fuse(accel, gyro);
        eul = euler(q1, 'ZYX', 'frame');
    end

    if (method ==2)
        madgwick = MadgwickAHRS('SamplePeriod', dt, 'Beta', 0.1);
        q2 = zeros(length(time), 4);
        for t = 1:length(time)
            madgwick.UpdateIMU(gyro(t,:), accel(t,:));
            q2(t, :) = madgwick.Quaternion;
        end
        eul = euler(quaternion(q2), 'XYZ', 'frame');
    end

    if (method ==3)
        mahony = MahonyAHRS('SamplePeriod', dt, 'Kp', 0.5);
        q3 = zeros(length(time), 4);
        for t = 1:length(time)
            mahony.UpdateIMU(gyro(t,:), accel(t,:));
            q3(t, :) = mahony.Quaternion;
        end
        eul = euler(quaternion(q3), 'XYZ', 'frame');
    end

    if (method ==4)
        eul = LPFilter(filter(2*dt, [1 0 -1], HPFilter(gyro, fs, fh)), fs, fl);
    end
    eul(:,1) = eul(:,1) - mean(eul(:,1));
    eul(:,3) = eul(:,3) - mean(eul(:,3));
    aclz = accel(:, 3) - 9.81;  % Z Linear Acceleration (m/s^2)
    ZposAcHP=filter(dt^2,[1,-2,1],HPFilter(aclz, fs, 1)); % Double integrator with Highpass with 1 Hz cutoff
    trajectory = zeros(n,7);
    trajectory(:,1) = time;
    trajectory(:,4) = ZposAcHP;
    trajectory(:,5:7) = eul;

end