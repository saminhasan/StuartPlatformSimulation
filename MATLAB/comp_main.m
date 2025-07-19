clc; close all; clear all; %#ok<CLALL>
[r, n, rB, dB, rP, dP] = deal(0.1, 3.5878,0.1161, 0.106/2, 0.0716025403784439,  0.02);
[trajectory, tf, ts] = genPose();
pbs  = (0.3:0.1:1.0)';
sim_no    = size(pbs,1);
outs = cell(1,sim_no);           % preallocate
params
homezz = zeros(sim_no,1);
for i = 1:sim_no
    pb = pbs(i);
    rP = rB * pb;
    sp = StuartPlatform(r, n, rB, dB, rP, dP);
    homezz(i)= sp.homez;
    motorAngles = zeros(size(trajectory));
    for k = 1:size(trajectory,1)
        motorAngles(k,:) = [trajectory(k,1), sp.move(trajectory(k,2:7))'];
    end

    % run model and stash the SimulationOutput object
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
        plot(t(20:end), tau(20:end), 'Color', clr(2*i,:), 'DisplayName',sprintf('ratio = %.1f', pb))
        % plot rated/peak lines
        % yline( peak_torque, '--r','Peak','HandleVisibility','off','LabelVerticalAlignment','top');
        % yline(-peak_torque,'--r','Peak','HandleVisibility','off','LabelVerticalAlignment','bottom');
        % yline( rated_torque, '--g','Rated','HandleVisibility','off','LabelVerticalAlignment','top');
        % yline(-rated_torque,'--g','Rated','HandleVisibility','off','LabelVerticalAlignment','bottom');
        % xlabel('Time (s)')
        % ylabel('Torque (Nm)')
        hold off; legend;grid on;
    end
    title(sprintf('Motor - %d', k));
end

poseNames = {'X', 'Y', 'Z', 'roll', 'pitch', 'yaw'};
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
        plot(tsim(20:end), rs(20:end), 'Color', clr(i,:), 'DisplayName', sprintf('Simscape %s | %f', poseNames{k}, pb));
        plot(time(3:end), r(3:end), 'Color', clr(i,:), 'DisplayName', sprintf('Setpoint %s| %f', poseNames{k}, pb), 'LineStyle','-.');
        xlabel('Time (s)');
        legend;
        grid on;
        grid minor;
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
    r = filter([1,-2,1],ts^2,trajectory(:,4));
    hold on;
    plot(tsim(20:end), rs(20:end), 'Color', clr(i,:), 'DisplayName', sprintf('Simscape %s | %f', 'Zacc', pb));
    plot(time(3:end), r(3:end), 'Color', clr(i,:), 'DisplayName', sprintf('Setpoint %s| %f', 'Zacc', pb), 'LineStyle','-.');
    xlabel('Time (s)');
    legend;
    grid on;
    grid minor;
    hold off;
end
title(sprintf('%s', poseNames{k}));