function [] = plotMotorInfo(out)
params
colors = ['r', 'g', 'b', 'c', 'm', 'y'];


%retrive simulation data
sim_time = out.simout.Time;
thetas = out.simout.Data(:, (2:4:22)-1);
omegas = out.simout.Data(:, 2:4:22);
alphas = out.simout.Data(:, (2:4:22)+1); %#ok<NASGU>
taus_load = out.simout.Data(:, (2:4:22) + 2);
omegas_rad_motor = omegas * N; % Angular velocity in rad/s in motor frame
tau_motor =  taus_load / N; % torque in motor frame
power_motor = tau_motor .* omegas_rad_motor;
tau_motor_rms = sqrt((tau_motor.^2)/length(tau_motor)); %#ok<NASGU>
omegas_rpm_motor = omegas * (60 / (2 * pi))* N;

% Compute max values
[max_power] = max(max(power_motor));
[max_tau] = max(max(tau_motor));
[max_rad] = max(max(omegas_rad_motor));
% Bench test - at 36 V, max RPM(motor frame) =  981 rpm -> 103 rad/s
% Kt = 36/103  = 36/103 Nm/A = 36/103 V/ (rad/s)
Kt = 0.28; % Nm/A from datasheet to be on the safe side.
max_current = max_tau / Kt;




% Print results
fprintf('Max Power: %.6f W\n', max_power);
fprintf('Max Dynamic Torque: %.6f Nm \n', max_tau);
fprintf('Max RPM (Motor Frame): %.6f rad/s\n', max_rad/ 9.549297);
fprintf('Max Current: %.6f A\n', max_current);

% plot motor T omega Nm and RPM
figure('Name', 'Torque (Nm) vs Angular Velocity (RPM)', 'NumberTitle', 'off');
hold on;
plots = gobjects(1, 6); 
labels = cell(1, 6);
for i = 1:6
    % Plot data for each motor
    plots(i) = plot(omegas_rpm_motor(:, i), tau_motor(:, i), 'Color', colors(i));
    % plots(i) = plot(masked_omegas_rpm(:, i) * N, masked_t_total(:, i), 'Color', colors(i));
    labels{i} = ['Motor ' num2str(i)]; % Store labels in a cell array
end

% Add legend with motor labels
legend(plots, labels, 'Location', 'northwest');

yline(peak_torque, '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment','top');
yline(rated_torque, '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment','top');
yline(-rated_torque, '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment','bottom');
yline(-peak_torque, '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment','bottom');

xlabel('Angular Velocity (RPM)');
ylabel('Torque (Nm)');
title('Torque (Nm) vs Angular Velocity (RPM)');
grid on;grid minor;
hold off;

figure('Name', 'Torque (Nm) vs Time (s)', 'NumberTitle', 'off');
hold on;
time_plots = gobjects(1, 6); 
for i = 1:6
    % Plot torque vs time for each motor
    time_plots(i) = plot(sim_time, tau_motor(:, i), 'Color', colors(i));
    labels{i} = ['Motor ' num2str(i)]; % Store labels in a cell array
end

% Add legend with motor labels
legend(time_plots, labels, 'Location', 'northwest');

% Add y-lines for motor peaks and rated values
yline(peak_torque, '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment','top');
yline(rated_torque, '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment','top');
yline(-rated_torque, '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment','bottom');
yline(-peak_torque, '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment','bottom');

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Torque (Nm) vs Time (s)');
grid on;grid minor;
hold off;


figure('Name', 'Torque (Nm) vs Theta (degrees)', 'NumberTitle', 'off');
hold on;
time_plots = gobjects(1, 6); 
for i = 1:6
    % Plot torque vs time for each motor
    time_plots(i) = plot(rad2deg(thetas(:, i)), tau_motor(:, i), 'Color', colors(i));
    labels{i} = ['Motor ' num2str(i)]; % Store labels in a cell array
end

% Add legend with motor labels
legend(time_plots, labels, 'Location', 'northwest');

% Add y-lines for motor peaks and rated values
yline(peak_torque, '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment','top');
yline(rated_torque, '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment','top');
yline(-rated_torque, '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment','bottom');
yline(-peak_torque, '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment','bottom');

xlabel('Theta (degrees)');
ylabel('Torque (Nm)');
title('Torque (Nm) vs Theta (degrees');
grid on;
grid minor;
hold off;
