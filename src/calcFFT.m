function [f, P1] = calcFFT(t, x)
% onesided_fft_simple  One-sided FFT of a real signal (no windows/filters).
% Inputs:  t (time vector), x (signal)
% Outputs: f (Hz), P1 (one-sided amplitude spectrum)

    t = t(:); x = x(:);
    Fs = 1/mean(diff(t));     % sampling frequency (assumes uniform sampling)

    x0 = x - mean(x);         % remove DC
    N  = numel(x0);
    Y  = fft(x0);
    P2 = abs(Y/N);            % two-sided spectrum
    L  = floor(N/2);
    P1 = P2(1:L+1);           % one-sided spectrum
    if L >= 1
        P1(2:end-1) = 2*P1(2:end-1);
    end
    f  = (0:L)'*(Fs/N);       % frequency axis in Hz
end
