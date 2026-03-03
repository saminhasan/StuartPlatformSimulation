
fh = 0.5;
fl = 15;
file_number = 1;
DATA_PATH = fullfile(pwd, 'IMUData');
filelist = string({dir(fullfile(DATA_PATH, '*.mat')).name});
data = load(fullfile(DATA_PATH, filelist(file_number)));
imu_data = data.data_mtl;% Data is saved in matrix name data_mtl
n = length(imu_data);
plotIMUData(imu_data);
time = imu_data(:, 1); % Time (s)
time(:, 1) = time(:, 1) - time(1,1); % Normalize time to start at zero
dt = mean(diff(time));
fs = 1/ dt;
accelZ = imu_data(:,4);


% matlab_accel_bias.m
fname = 'data.txt';
rest_n = 685;
g = 9.81;

% load rows starting with '|' -> [ax, ay, az]
fid = fopen(fname,'r'); assert(fid>0,'Cannot open file');
data = [];
while true
    t = fgetl(fid);
    if ~ischar(t), break; end
    if ~isempty(t) && t(1) == '|'
        v = sscanf(t(2:end), '%f,%f,%f', [3,1]);
        if numel(v) == 3, data(end+1,:) = v.'; end %#ok<AGROW>
    end
end
fclose(fid);

assert(size(data,1) > rest_n && size(data,2) == 3, 'Bad data or too few samples');

% bias so rest -> [0, 0, 9.81]
rest_mean = mean(data(1:rest_n,:), 1);
bias = rest_mean - [0 0 g];

% apply bias removal
corr = data - bias;

fprintf('Rest mean: [%.6f %.6f %.6f]\n', rest_mean);
fprintf('Bias (subtracting this): [%.6f %.6f %.6f]\n', bias);

% plot Z after rest_n
z = corr(rest_n+1:end, 3);


% Option B — reset time to 0 at the end of the rest window:
tz = (0:numel(z)-1)*dt;
figure;
hold on;
plot(time, accelZ, DisplayName='ref');
plot(tz, z, DisplayName='act');
grid on;
xlabel('Time(s)');
ylabel('Acceleration (m/s^2)');
title('Z-axis Acceleration vs Time');
legend
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

% % Simple FFT demo: three sine waves + one-sided FFT
% clear; close all; clc;
% 
% % ---- Setup: sampling ----
% dt = 1e-3;
% fs = 1/dt;
% tf = 5;
% t = 0:dt:10-dt;  % Time vector for 1 second duration
% L = length(t);  % Length of the signal
% Fs = fs;        % Sampling frequency
% % ---- Signals: three sines ----
% A1 = 1.0; f1 = 1;      s1 = A1*sin(2*pi*f1*t + pi/2);
% A2 = 0.5; f2 = 10;     s2 = A2*sin(2*pi*f2*t + pi /3);
% A3 = 2.0; f3 = 100;    s3 = A3*sin(2*pi*f3*t + pi /3);
% 
% x = s1 + s2 + s3;      % summed signal
% 
% % ---- Plot time-domain signals ----
% figure('Name','Time-domain signals','NumberTitle','off');
% plot(t, s1, 'DisplayName','1 Hz'); hold on;
% plot(t, s2, 'DisplayName','10 Hz');
% plot(t, s3, 'DisplayName','100 Hz');
% plot(t, x, 'DisplayName','Sum'); hold off;
% xlabel('Time (s)'); ylabel('Amplitude');
% title('Sine components and their sum');
% legend('Location','best'); grid on;
% 
% [f, P1] = calcFFT(t, x);
% 
% figure('Name','One-sided amplitude spectrum','NumberTitle','off');
% plot(f, P1, 'Marker','none');
% xlabel('Frequency (Hz)'); ylabel('|X(f)|');
% title('One-sided FFT of summed signal');
% xlim([0 150]); grid on;