function filteredSignal = HPFilter(signal, fs, fc)
    w1 = 2 * pi * fc; % Angular cutoff frequency
    zeta1 = 0.707;
    Bw = [1, 0, 0, 0];
    Aw = conv([1, w1], [1, 2 * zeta1 * w1, w1^2]);
    Gz = c2d(tf(Bw, Aw), 1/fs);
    Bz = Gz.Numerator{1};
    Az = Gz.Denominator{1};
    filteredSignal = filter(Bz, Az, signal);
end