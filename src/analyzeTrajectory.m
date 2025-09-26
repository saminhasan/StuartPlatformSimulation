function results = analyzeTrajectory(trajectory)
% analyzeTrajectory  Extract motion features and fundamental frequencies from trajectory.
%   trajectory: [N x 7] double -> columns = [t, x, y, z, rx, ry, rz]%
%   Outputs: results struct and a LaTeX table printed to console.

    assert(isfloat(trajectory) && size(trajectory,2)==7, 'trajectory must be [N x 7] double');

    % --- Parse & basic checks ---
    t  = trajectory(:,1); t = t(:);
    % x  = trajectory(:,2); y = trajectory(:,3);
    z = trajectory(:,4);
    rx = trajectory(:,5); ry = trajectory(:,6); rz = trajectory(:,7); % radians



    % Sampling frequency
    dt  = median(diff(t));
    fs  = 1/dt;
    N   = numel(t);
    assert(N > 10, 'Not enough samples.');

    % Helper: center signal (remove mean)
    c = @(s) s - mean(s);

    % ------------------------------
    % Fundamental frequency via FFT
    % ------------------------------
    % Use Hann window + zero padding, search in [0.3, fs/2] Hz (ignore DC)
    fmin = 0.3; fmax = fs/2;
    ffun = @(s) fundamental_fft(c(s), fs, fmin, fmax);

    fz   = ffun(z);
    frx  = ffun(rx);
    fry  = ffun(ry);
    frz  = ffun(rz);

    % ---------------------------------
    % Amplitudes / bounds (position/rot)
    % ---------------------------------
    % Use centered signals ->
    amp = @(s) max(abs(c(s)));              % half peak-to-peak
    z_amp_m      = amp(z);                  % meters
    rx_amp_deg   = rad2deg(amp(rx));        % degrees
    ry_amp_deg   = rad2deg(amp(ry));
    rz_amp_deg   = rad2deg(amp(rz));

    % --------------------------
    % Vertical linear acceleration
    % --------------------------
    % Compute a clean second derivative of z(t):
    %  - Savitzky–Golay smoothing derivative (order 3, frame ~41 ms)
    %  - Fallback to central differences if sgolayfilt unavailable
    try
        win = max(11, 2*floor(0.02*fs)+1);  % ~20 ms odd window
        z_s  = sgolayfilt(z, 3, win);       % smooth position
        vz   = gradient(z_s, dt);
        az   = gradient(vz, dt);
    catch
        % No Signal Processing Toolbox: use plain gradient (still decent)
        vz   = gradient(z, dt);
        az   = gradient(vz, dt);
        % light zero-phase low-pass to tame noise (Butterworth 25 Hz)
        [b,a] = butter(3, min(25,0.45*fs)/(fs/2), 'low');
        az    = filtfilt(b,a,az);
    end

    g0 = 9.80665;
    az_g = az / g0;
    az_g_min = min(az_g);
    az_g_max = max(az_g);

    % --------------------------
    % Pack results
    % --------------------------
    results.fs = fs;
    results.duration_s = t(end) - t(1);

    results.translation.Z.pm_cm   = 100*z_amp_m;   % ± in cm
    results.translation.Z.fund_Hz = fz;

    results.rotation.Z.pm_deg     = rz_amp_deg;    % yaw (transverse)
    results.rotation.Z.fund_Hz    = frz;

    results.rotation.Y.pm_deg     = ry_amp_deg;    % pitch (sagittal)
    results.rotation.Y.fund_Hz    = fry;

    results.rotation.X.pm_deg     = rx_amp_deg;    % roll (frontal)
    results.rotation.X.fund_Hz    = frx;

    results.linAccZ.min_g         = az_g_min;
    results.linAccZ.max_g         = az_g_max;
    results.linAccZ.fund_Hz       = ffun(az);      % often aligns with gait freq

    % --------------------------
    % Print LaTeX table (format)
    % --------------------------
    fprintf('\n%% LaTeX table: Torso Motion Dataset (auto-generated)\n');
    fprintf('\\begin{table}[h]\n\\centering\n\n');
    fprintf('\\begin{tabular}{cccc}\\hline\n');
    fprintf('\\textbf{Feature} & \\textbf{Axis} & \\textbf{Value} & \\textbf{Fundamental Frequency (Hz)} \\\\\\hline\n');

    fprintf('Translation & Z & $\\pm$%.1f cm & %.2f \\\\\\hline\n', ...
        results.translation.Z.pm_cm, results.translation.Z.fund_Hz);

    fprintf('Rotation & Z (transverse) & $\\pm$%.1f$^\\circ$ & %.2f \\\\\\hline\n', ...
        results.rotation.Z.pm_deg, results.rotation.Z.fund_Hz);

    fprintf('Rotation & Y (sagittal) & $\\pm$%.1f$^\\circ$ & %.2f \\\\\\hline\n', ...
        results.rotation.Y.pm_deg, results.rotation.Y.fund_Hz);

    fprintf('Rotation & X (frontal) & $\\pm$%.1f$^\\circ$ & %.2f \\\\\\hline\n', ...
        results.rotation.X.pm_deg, results.rotation.X.fund_Hz);

    fprintf('Linear Acceleration & Z & %.1fg to %.1fg & %.2f \\\\\\hline\n', ...
        results.linAccZ.min_g, results.linAccZ.max_g, results.linAccZ.fund_Hz);

    fprintf('\\end{tabular}\n\\caption{Torso Motion Dataset}\n\\label{tab:torso_motion}\n\\end{table}\n\n');

    % --------------------------
    %  plots
    % --------------------------
        % Spectra for z, rx, ry, rz
        figure('Name','Fundamental Frequency (Spectra)','NumberTitle','off');
        sigs  = {z, rx, ry, rz};
        names = {'Z (m)', 'Rx (rad)', 'Ry (rad)', 'Rz (rad)'};
        for k = 1:4
            subplot(2,2,k);
            [f, P] = onesided_spectrum(c(sigs{k}), fs);
            plot(f, P); grid on; xlim([0, min(10, fs/2)]);
            xlabel('Frequency (Hz)'); ylabel('|X(f)|');
            title(['Spectrum: ' names{k}]);
        end
end

% ======== helpers ========

function f0 = fundamental_fft(x, fs, fmin, fmax)
% fundamental_fft  Find dominant frequency peak excluding DC.
    x = x(:);
    x = x - mean(x);
    N  = numel(x);
    w  = hann(N);
    X  = fft(w .* x, 2^nextpow2(N));
    Nf = numel(X);
    f  = (0:Nf-1)' * (fs/Nf);
    P  = abs(X);

    % only positive freqs
    idx = f >= fmin & f <= fmax;
    if ~any(idx)
        f0 = NaN; return;
    end
    [~, iMax] = max(P(idx));
    fcand = f(idx);
    f0 = fcand(iMax);
end

function [f, P1] = onesided_spectrum(x, fs)
% onesided_spectrum  One-sided magnitude spectrum with Hann window.
    x = x(:) - mean(x);
    N  = numel(x);
    w  = hann(N);
    X  = fft(w .* x, 2^nextpow2(N));
    Nf = numel(X);
    f  = (0:Nf/2)' * (fs/Nf);
    X  = X(1:numel(f));
    P1 = abs(X);
end
