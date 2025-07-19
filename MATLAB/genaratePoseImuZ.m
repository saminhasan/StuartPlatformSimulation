function [pose, tf, ts]= genaratePoseImuZ()
    file_number = 1;
    DATA_PATH = 'C:\Users\james\OneDrive\Desktop\StuartPlatformSimulation\IMUData';%% change this address 
    filelist = string({dir(fullfile(DATA_PATH, '*.mat')).name});
    data = load(fullfile(DATA_PATH, filelist(file_number)));
    disp(['Filename: ', filelist(file_number)]);
    imu_data = data.data_mtl;% Data is saved in matrix name data_mtl
    n = length(imu_data);
    % plotIMUData(imu_data);
    imu_data(:, 1) = imu_data(:, 1) - imu_data(1,1); % Normalize time to start at zero
    time = imu_data(:, 1); % Time (s)
    dt = mean(diff(time)); 
    fs = 1/ dt;
    tf = time(end);
    ts = dt;
    acl = imu_data(:, 4) - 9.81;  % Z Linear Acceleration (m/s^2)
    % Double integrator with Highpass
    fa=1;
    acHP=HPFilter(acl,fs,fa);
    ZposAcHP=filter(dt^2,[1,-2,1],acHP);
    % d = load("z_m.txt");
    % plot(d - posAcHP);
    % plot(ZposAcHP)
    pose = zeros(n,7);
    pose(:,1) = time;
    pose(:,4) = ZposAcHP;
    % for i = 2:7
    %     pose(:,i) = signalWrapper(pose(:,1),pose(:,i)); %adding 
    % end
end