for i = 1:sim_no
    pb = pbs(i); % N, link length, its not really a len, but rather a design param
    out = outs{i};
    t = out.simout.Time;
    sim_time = out.simout.Time;
    thetas = out.simout.Data(:, (2:4:22)-1);
    omegas = out.simout.Data(:, 2:4:22);
    alphas = out.simout.Data(:, (2:4:22)+1);
    taus_load = out.simout.Data(:, (2:4:22) + 2);
    %  for all six motors, so shape is n, 6 for each of them
end