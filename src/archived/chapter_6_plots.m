clc; close all; clear all; %#ok<CLALL>
    addpath(fullfile(pwd,"quaternion_library"));
    fh = 0.5;
    fl = 50;
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


    eul = LPFilter(filter(2*dt, [1 0 -1], HPFilter(gyro, fs, fh)), fs, fl);
    eul(:,1) = eul(:,1) - mean(eul(:,1));
    eul(:,3) = eul(:,3) - mean(eul(:,3));
    aclz = accel(:, 3) - 9.81;  % Z Linear Acceleration (m/s^2)
    ZposAcHP=filter(dt^2,[1,-2,1],HPFilter(aclz, fs, 1)); % Double integrator with Highpass with 1 Hz cutoff
    trajectory = zeros(n,7);
    trajectory(:,1) = time;
    trajectory(:,4) = ZposAcHP;
    trajectory(:,5:7) = eul;
    acczR = filter([1,-2,1], dt^2, ZposAcHP);
    ZposAcHP=filter(dt^2,[1,-2,1],HPFilter(aclz, fs, 1)); % Double integrator with Highpass with 1 Hz cutoff
    figure('Name','Single-Sided Amplitude Spectrum of Z acceleration ','NumberTitle','off');
    [f2, P2] = calcFFT(time, ZposAcHP);
    hold on;
    semilogx(f2, P2);
    hold off;
    % legend;
    grid on; grid minor
    xlim([f2(2), f2(end)]);
    ylabel('Amplitude'); xlabel('Frequency (Hz)');















zeta = 1/sqrt(2);
wn_list = 3:3:300;    % natural frequencies (Hz)
Ts = 1e-3;           % sampling time
t = (0:length(ZposAcHP)-1)*Ts;

peak_acc = zeros(size(wn_list));

s = tf('s');

for i = 1:length(wn_list)
    wn = 2*pi*wn_list(i);   % rad/s
    % Define 2nd order system in continuous time
    H = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);
    % Simulate system response
    Acczy = lsim(H, aclz, t);
    % Record peak absolute value
    peak_acc(i) = max(abs(Acczy));
end

% Plot results
figure;
plot(wn_list, peak_acc, '-o'); hold on;
yline(max(abs(aclz)), 'r--', 'Max raw acc');
grid on;
xlabel('Natural frequency (Hz)');
ylabel('Peak Acceleration(m/s^2)');
title('Peak Acceleration vs Natural Frequency');
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
