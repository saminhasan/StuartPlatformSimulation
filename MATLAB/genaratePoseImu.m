function [pose, tf, ts]= genaratePoseImu(num_cycle) %#ok<INUSD>
    fh = 0.5;
    fl = 50;
    method = 1;
    file_number = 1;
    DATA_PATH = 'C:\Users\james\OneDrive\Desktop\StuartPlatformSimulation\MATLAB\IMUData';%% change this address
    filelist = string({dir(fullfile(DATA_PATH, '*.mat')).name});
    data = load(fullfile(DATA_PATH, filelist(file_number)));
    % disp(['Filename: ', filelist(file_number)]);
    imu_data = data.data_mtl;% Data is saved in matrix name data_mtl
    n = length(imu_data);
    % plotIMUData(imu_data);
    imu_data(:, 1) = imu_data(:, 1) - imu_data(1,1); % Normalize time to start at zero
    time = imu_data(:, 1); % Time (s)
    ts = mean(diff(time));
    fs = 1/ ts;
    tf = time(end);
    accel = imu_data(:,2:4);
    gyro  = deg2rad(imu_data(:,5:7));
    if (method ==4)

        yaw = LPFilter(filter(2*ts, [1 0 -1], HPFilter(gyro(:,3), fs, fh)), fs, fl);
        pitch = LPFilter(filter(2*ts, [1 0 -1], HPFilter(gyro(:,2), fs, fh)), fs, fl);
        roll = LPFilter(filter(2*ts, [1 0 -1], HPFilter(gyro(:,1), fs, fh)), fs, fl);
        eul = rad2deg([roll, pitch, yaw]);
    end
    if (method ==1)
        fuse = imufilter('SampleRate', fs);
        q1 = fuse(accel, gyro);
        eul = eulerd(q1, 'ZYX', 'frame');
        eul(:,1) = LPFilter(eul(:,1), fs, fl);
        eul(:,2) = LPFilter(eul(:,2), fs, fl);
        eul(:,3) = LPFilter(eul(:,3), fs, fl);
    end
    if (method ==2)
        madgwick = MadgwickAHRS('SamplePeriod', ts, 'Beta', 0.1);
        q2 = zeros(length(time), 4);
        for t = 1:length(time)
            madgwick.UpdateIMU(gyro(t,:), accel(t,:));
            q2(t, :) = madgwick.Quaternion;
        end
        eul = eulerd(quaternion(q2), 'ZYX', 'frame');
    end
    if (method ==3)
        mahony = MahonyAHRS('SamplePeriod', ts, 'Kp', 0.5);
        q3 = zeros(length(time), 4);
        for t = 1:length(time)
            mahony.UpdateIMU(gyro(t,:), accel(t,:));
            q3(t, :) = mahony.Quaternion;
        end
        eul = eulerd(quaternion(q3), 'ZYX', 'frame');
    end
    eul(:,1) = eul(:,1) - mean(eul(:,1));
    eul(:,3) = eul(:,3) - mean(eul(:,3));
    aclz = imu_data(:, 4) - 9.81;  % Z Linear Acceleration (m/s^2)
    % Double integrator with Highpass
    fhz=1;
    ZposAcHP=filter(ts^2,[1,-2,1],HPFilter(aclz,fs,fhz));
    pose = zeros(n,7);
    pose(:,1) = time;
    pose(:,4) = ZposAcHP;
    pose(:,7) = deg2rad(eul(:,3));
    pose(:,6) = deg2rad(eul(:,2));
    pose(:,5) = deg2rad(eul(:,1));
end