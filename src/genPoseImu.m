function trajectory = genPoseImu(num_cycle) %#ok<INUSD>
    addpath(fullfile(pwd,"quaternion_library"));
    fh = 0.5;
    fl = 15;
    method = 1;
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
    acczR = filter([1,-2,1], dt^2, ZposAcHP);
    % figure('Name','IMU Data','NumberTitle','off');
    % [f1, P1] = calcFFT(time, acczR);
    % [f2, P2] = calcFFT(time, aclz);
    % 
    % % --- dB conversion (20*log10 of magnitude), shared ref across both traces
    % ref_acc = max([P1(:); P2(:)]);
    % P1dB = 20*log10((P1 + eps)/ref_acc);
    % P2dB = 20*log10((P2 + eps)/ref_acc);
    % 
    % hold on;
    % plot(f2, P2dB, 'DisplayName','Raw Acceleration');
    % plot(f1, P1dB, 'DisplayName','High Pass Filtered Acceleration');
    % hold off;
    % legend; grid on; grid minor
    % set(gca,'XScale','log');
    % xlim([0.5, fs/2]);
    % ylabel('Magnitude (dB, ref = peak)'); xlabel('Frequency (Hz)');

    % --- angular rates from orientation (velocity filter)
    % phi_dot   = filter([1 0 -1], 2*dt, eul(:,1));
    % theta_dot = filter([1 0 -1], 2*dt, eul(:,2));
    % psi_dot   = filter([1 0 -1], 2*dt, eul(:,3));

    % % ---- FFT comparisons: recovered vs raw ----
    % % Roll (x)
    % [fp,  Pp]  = calcFFT(time, phi_dot);
    % [fgx, Pgx] = calcFFT(time, gyro(:,1));
    % ref_p = max([Pp(:); Pgx(:)]);
    % Pp_dB  = 20*log10((Pp  + eps)/ref_p);
    % Pgx_dB = 20*log10((Pgx + eps)/ref_p);
    % 
    % figure('Name','FFT: Roll rate','NumberTitle','off');
    % hold on;
    % plot(fgx, Pgx_dB, 'DisplayName','gyro p (raw)');
    % plot(fp,  Pp_dB,  'DisplayName','d\phi/dt (from eul)');
    % hold off;
    % legend; grid on; grid minor; xlabel('Frequency (Hz)'); ylabel('Magnitude (dB, ref = peak)'); title('Roll rate (p)');
    % set(gca,'XScale','log'); xlim([0.5, fs/2]);
    % 
    % % Pitch (y)
    % [fq,  Pq]  = calcFFT(time, theta_dot);
    % [fgy, Pgy] = calcFFT(time, gyro(:,2));
    % ref_q = max([Pq(:); Pgy(:)]);
    % Pq_dB  = 20*log10((Pq  + eps)/ref_q);
    % Pgy_dB = 20*log10((Pgy + eps)/ref_q);
    % 
    % figure('Name','FFT: Pitch rate','NumberTitle','off');
    % hold on;
    % plot(fgy, Pgy_dB, 'DisplayName','gyro q (raw)');
    % plot(fq,  Pq_dB,  'DisplayName','d\theta/dt (from eul)');
    % hold off;
    % legend; grid on; grid minor; xlabel('Frequency (Hz)'); ylabel('Magnitude (dB, ref = peak)'); title('Pitch rate (q)');
    % set(gca,'XScale','log'); xlim([0.5, fs/2]);
    % 
    % % Yaw (z)
    % [fr,  Pr]  = calcFFT(time, psi_dot);
    % [fgz, Pgz] = calcFFT(time, gyro(:,3));
    % ref_r = max([Pr(:); Pgz(:)]);
    % Pr_dB  = 20*log10((Pr  + eps)/ref_r);
    % Pgz_dB = 20*log10((Pgz + eps)/ref_r);
    % 
    % figure('Name','FFT: Yaw rate','NumberTitle','off');
    % hold on;
    % plot(fgz, Pgz_dB, 'DisplayName','gyro r (raw)');
    % plot(fr,  Pr_dB,  'DisplayName','d\psi/dt (from eul)');
    % hold off;
    % legend; grid on; grid minor; xlabel('Frequency (Hz)'); ylabel('Magnitude (dB, ref = peak)'); title('Yaw rate (r)');
    % set(gca,'XScale','log'); xlim([0.5, fs/2]);
end