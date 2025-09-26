% Simple FFT demo: three sine waves + one-sided FFT
clear; close all; clc;

% ---- Setup: sampling ----
dt = 1e-3;
fs = 1/dt;
tf = 5;
t = 0:dt:10-dt;  % Time vector for 1 second duration
L = length(t);  % Length of the signal
Fs = fs;        % Sampling frequency
% ---- Signals: three sines ----
A1 = 1.0; f1 = 1;      s1 = A1*sin(2*pi*f1*t + pi/2);
A2 = 0.5; f2 = 10;     s2 = A2*sin(2*pi*f2*t + pi /3);
A3 = 2.0; f3 = 100;    s3 = A3*sin(2*pi*f3*t + pi /3);

x = s1 + s2 + s3;      % summed signal

% ---- Plot time-domain signals ----
figure('Name','Time-domain signals','NumberTitle','off');
plot(t, s1, 'DisplayName','1 Hz'); hold on;
plot(t, s2, 'DisplayName','10 Hz');
plot(t, s3, 'DisplayName','100 Hz');
plot(t, x, 'DisplayName','Sum'); hold off;
xlabel('Time (s)'); ylabel('Amplitude');
title('Sine components and their sum');
legend('Location','best'); grid on;

[f, P1] = calcFFT(t, x);

figure('Name','One-sided amplitude spectrum','NumberTitle','off');
plot(f, P1, 'Marker','none');
xlabel('Frequency (Hz)'); ylabel('|X(f)|');
title('One-sided FFT of summed signal');
xlim([0 150]); grid on;