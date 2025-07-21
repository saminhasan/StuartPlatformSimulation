clc; close all; clear all; %#ok<CLALL>
params
trajectory = genPoseSynthetic(5);
time = trajectory(:,1);
tf = time(end);
pbs  = (0.3:0.1:1.0)';
sim_no    = size(pbs,1);
outs = cell(1,sim_no);           % preallocate
homezz = zeros(sim_no,1);
for i = 1:sim_no
    pb = pbs(i);
    rP = rB * pb;
    sp = StuartPlatform(r, n, rB, dB, rP, dP);
    homezz(i)= sp.homez;
    jointAngles = sp.move(trajectory);
    outs{i} = sim("SP.slx");
end
clr = generateColorsHSV(sim_no*2);
for k = 1:6
    figure('Name', sprintf('Motor - %.1d', k), 'NumberTitle','off');
    for i = 1:sim_no
        pb = pbs(i);
        out = outs{i};
        t = out.simout.Time;
        tau_motor = out.simout.Data(:, (2:4:22)+2);  % load torque
        tau = tau_motor(:,k);
        hold on;
        plot(t, tau, 'Color', clr(2*i,:), 'DisplayName',sprintf('ratio = %.1f', pb))
        hold off; legend; grid on;
    end
    xlabel('Time (s)');
    ylabel(sprintf('\\tau_{motor %d} (Nm)', k));
    title(sprintf('Motor - %d', k));
end

poseNames = {'X', 'Y', 'Z', 'roll', 'pitch', 'yaw'};
poseUnits = {'(m)', '(m)', '(m)', '(\circ)', '(\circ)', '(\circ)'};
for k = 1:6
    figure('Name', sprintf('%s', poseNames{k}), 'NumberTitle','off');
    for i = 1:sim_no
        pb = pbs(i);
        out = outs{i};
        tsim = out.pose_simscape.Time;
        rs = out.pose_simscape.Data(:,k);
        time = trajectory(:,1);
        r = trajectory(:,k+1);
        if (k+1==4)
            r = r - mean(r);
            rs = rs - mean(rs);
        end

        hold on;
        plot(tsim, rs, 'Color', clr(i,:), 'DisplayName', sprintf('Simscape %s | %f', poseNames{k}, pb));
        plot(time+riseDealy, r, 'Color', clr(i,:), 'DisplayName', sprintf('Setpoint %s| %f', poseNames{k}, pb), 'LineStyle','-.');
        xlabel('Time (s)');
        ylabel(sprintf('%s %s', poseNames{k}, poseUnits{k}));
        legend;
        grid on; grid minor;
        hold off;
    end
    title(sprintf('%s', poseNames{k}));
end

f = figure('Name', sprintf('%s', 'Zacc'), 'NumberTitle','off');
for i = 1:sim_no
    pb = pbs(i);
    out = outs{i};
    tsim = out.pose_simscape.Time;
    rs = out.pose_simscape.Data(:,7);
    time = trajectory(:,1);
    r = filter([1,-2,1],dt^2,trajectory(:,4));
    hold on;
    plot(tsim, rs, 'Color', clr(i,:), 'DisplayName', sprintf('Simscape %s | %f', 'Zacc', pb));
    plot(time(3:end)+riseDealy, r(3:end), 'Color', clr(i,:), 'DisplayName', sprintf('Setpoint %s| %f', 'Zacc', pb), 'LineStyle','-.');
    xlabel('Time (s)');
    ylabel('Z Acc (m/s^2)');
    legend;
    grid on; grid minor;
    hold off;
end
title(sprintf('%s', poseNames{k}));
