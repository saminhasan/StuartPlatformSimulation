function [] = plotPlatformInfo(out, trajectory, sp)
    params
    % Extract data from simulation output
    tsim = out.pose_simscape.Time;
    xsim = out.pose_simscape.Data(:,1) - sp.xshift;
    ysim = out.pose_simscape.Data(:,2);
    zsim = out.pose_simscape.Data(:,3)- sp.zshift;
    Rxsim = out.pose_simscape.Data(:,4);
    Rysim = out.pose_simscape.Data(:,5);
    Rzsim = out.pose_simscape.Data(:,6);
    azsim = out.pose_simscape.Data(:,7);

    % Unpack values from the pose struct array simulation input
    time = trajectory(:,1);
    x = trajectory(:,2);
    y = trajectory(:,3);
    z = trajectory(:,4) + sp.homez(3);

    Rx = trajectory(:,5);
    Ry = trajectory(:,6);
    Rz = trajectory(:,7);

    
    % figure('Name', 'pose vs Time (s)', 'NumberTitle', 'off');
    % 
    % subplot(3,2,1);
    % hold on;
    % plot(time, x, '-b', 'DisplayName', 'Setpoint X');
    % plot(tsim, xsim, '-r', 'DisplayName', 'Simscape X');
    % 
    % title('X Position');
    % xlabel('Time (s)');
    % ylabel('X Position (m)');
    % legend;
    % grid on;
    % grid minor;
    % hold off;
    % 
    % subplot(3,2,2);
    % hold on;
    % plot(tsim, ysim, '-r', 'DisplayName', 'Simscape Y');
    % plot(time, y, '-b', 'DisplayName', 'Setpoint Y');
    % title('Y Position');
    % xlabel('Time (s)');
    % ylabel('Y Position (m)');
    % legend;
    % grid on;
    % grid minor;
    % hold off;
    % 
    % subplot(3,2,3);
    % hold on;
    % plot(time, z, '-b', 'DisplayName', 'Setpoint Z');
    % plot(tsim, zsim, '-r', 'DisplayName', 'Simscape Z');
    % title('Z Position');
    % xlabel('Time (s)');
    % ylabel('Z Position (m)');
    % legend;
    % grid on;
    % grid minor;
    % hold off;
    % 
    % subplot(3,2,4);
    % hold on;
    % plot(time, rad2deg(Rx), '-b', 'DisplayName', 'Setpoint Rx');
    % plot(tsim, rad2deg(Rxsim), '-r', 'DisplayName', 'Simscape Rx');
    % title('Rx Orientation');
    % xlabel('Time (s)');
    % ylabel('Roll (deg)');
    % legend;
    % grid on;
    % grid minor;
    % hold off;
    % 
    % subplot(3,2,5);
    % hold on;
    % plot(time, rad2deg(Ry), '-b', 'DisplayName', 'Setpoint Ry');
    % plot(tsim, rad2deg(Rysim), '-r', 'DisplayName', 'Simscape Ry');
    % title('Ry Orientation');
    % xlabel('Time (s)');
    % ylabel('Pitch (deg)');
    % legend;
    % grid on;
    % grid minor;
    % hold off;
    % 
    % subplot(3,2,6);
    % hold on;
    % plot(time, rad2deg(Rz), '-b', 'DisplayName', 'Setpoint Rz');
    % plot(tsim, rad2deg(Rzsim), '-r', 'DisplayName', 'Simscape Rz');
    % title('Rz Orientation');
    % xlabel('Time (s)');
    % ylabel('Yaw (deg)');
    % legend;
    % grid on;
    % grid minor;
    % hold off;
% Pose vs Time — Setpoint vs Simscape
figure('Name','Pose vs Time','NumberTitle','off'); clf;

% LaTeX everywhere
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% Precompute degrees
Rx_d = rad2deg(Rx); Ry_d = rad2deg(Ry); Rz_d = rad2deg(Rz);
Rxsim_d = rad2deg(Rxsim); Rysim_d = rad2deg(Rysim); Rzsim_d = rad2deg(Rzsim);

vars_set = {x, y, z, Rx_d, Ry_d, Rz_d};
vars_sim = {xsim, ysim, zsim, Rxsim_d, Rysim_d, Rzsim_d};

titles = {...
    '$x$ position (m)', ...
    '$y$ position (m)', ...
    '$z$ position (m)', ...
    '$R_x$ roll (deg)', ...
    '$R_y$ pitch (deg)', ...
    '$R_z$ yaw (deg)'};

ylabels = {'Position (m)','Position (m)','Position (m)', ...
           'Angle (deg)','Angle (deg)','Angle (deg)'};

tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

for k = 1:6
    nexttile; hold on;
    plot(time, vars_set{k}, '-b', 'DisplayName','Setpoint');
    plot(tsim, vars_sim{k}, '-r','DisplayName','Simscape');
    title(titles{k});
    xlabel('Time (s)');
    ylabel(ylabels{k});
    grid on; grid minor; box on;
    legend('Location','northwest');
end

% sgtitle('Pose vs Time — Setpoint vs Simscape','Interpreter','latex');
    figure('Name', 'acceleration Z vs Time (s)', 'NumberTitle', 'off');
    hold on;

    Ts = mean(diff(time));
    ddz = filter([1,-2,1],Ts^2,z);
    plot(time (3:end) ,ddz(3:end)/g, '-b', 'DisplayName', 'Setpoint accZ');
    plot(tsim(3:end) , azsim(3:end)/g, '-r', 'DisplayName', 'Simscape accZ');
    title('Z axis acceleration');
    xlabel('Time (s)');
    ylabel('acceleration Z(g)');
    legend;
    grid on;
    grid minor;
    hold off;
    fprintf("Peak acceleration: Setpoint = %.3fg, Actual = %.3fg\n", max(ddz(3:end)/9.81), max(azsim(3:end)/9.81));

    
    % figure('Name', 'AccZ (m/s^2) vs Theta (degrees)', 'NumberTitle', 'off');
    % colors = ['r', 'g', 'b', 'c', 'm', 'y'];
    % thetas = out.simout.Data(:, (2:4:22)-1);
    % hold on;
    % time_plots = gobjects(1, 6); 
    % labels = cell(1, 6);
    % for i = 1:6
    %     % Plot torque vs time for each motor
    %     time_plots(i) = plot(rad2deg(thetas(:, i)), azsim, 'Color', colors(i));
    %     labels{i} = ['Motor ' num2str(i)]; % Store labels in a cell array
    % end
    % 
    % % Add legend with motor labels
    % legend(time_plots, labels, 'Location', 'northwest');
    % 
    % xlabel('Theta (degrees)');
    % ylabel('AccZ (m/s^2)');
    % title('AccZ (m/s^2) vs Theta (degrees)');
    % grid on;
    % grid minor;
    % hold off;
end