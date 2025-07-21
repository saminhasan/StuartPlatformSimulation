function [] = plotPlatformInfo(out, trajectory, homeZ)
    params
    % Extract data from simulation output
    tsim = out.pose_simscape.Time;
    xsim = out.pose_simscape.Data(:,1);
    ysim = out.pose_simscape.Data(:,2);
    zsim = out.pose_simscape.Data(:,3);
    Rxsim = out.pose_simscape.Data(:,4);
    Rysim = out.pose_simscape.Data(:,5);
    Rzsim = out.pose_simscape.Data(:,6);
    azsim = out.pose_simscape.Data(:,7);

    % Unpack values from the pose struct array simulation input
    time = trajectory(:,1) + riseDealy;
    x = trajectory(:,2);
    y = trajectory(:,3);
    z = trajectory(:,4) + homeZ;

    Rx = trajectory(:,5);
    Ry = trajectory(:,6);
    Rz = trajectory(:,7);

    
    figure('Name', 'pose vs Time (s)', 'NumberTitle', 'off');

    subplot(3,2,1);
    hold on;
    plot(tsim, xsim, '-r', 'DisplayName', 'Simscape X');
    plot(time, x, '-b', 'DisplayName', 'Setpoint X');
    title('X Position');
    xlabel('Time (s)');
    ylabel('X Position (m)');
    legend;
    grid on;
    grid minor;
    hold off;

    subplot(3,2,2);
    hold on;
    plot(time, y, '-b', 'DisplayName', 'Setpoint Y');
    plot(tsim, ysim, '-r', 'DisplayName', 'Simscape Y');

    title('Y Position');
    xlabel('Time (s)');
    ylabel('Y Position (m)');
    legend;
    grid on;
    grid minor;
    hold off;

    subplot(3,2,3);
    hold on;
    plot(tsim, zsim, '-r', 'DisplayName', 'Simscape Z');
    plot(time, z, '-b', 'DisplayName', 'Setpoint Z');
    title('Z Position');
    xlabel('Time (s)');
    ylabel('Z Position (m)');
    legend;
    grid on;
    grid minor;
    hold off;

    subplot(3,2,4);
    hold on;
    plot(tsim, rad2deg(Rxsim), '-r', 'DisplayName', 'Simscape Rx');
    plot(time, rad2deg(Rx), '-b', 'DisplayName', 'Setpoint Rx');
    title('Rx Orientation');
    xlabel('Time (s)');
    ylabel('Rx Orientation (deg)');
    legend;
    grid on;
    grid minor;
    hold off;

    subplot(3,2,5);
    hold on;
    plot(tsim, rad2deg(Rysim), '-r', 'DisplayName', 'Simscape Ry');
    plot(time, rad2deg(Ry), '-b', 'DisplayName', 'Setpoint Ry');
    title('Ry Orientation');
    xlabel('Time (s)');
    ylabel('Ry Orientation (deg)');
    legend;
    grid on;
    grid minor;
    hold off;

    subplot(3,2,6);
    hold on;
    plot(tsim, rad2deg(Rzsim), '-r', 'DisplayName', 'Simscape Rz');
    plot(time, rad2deg(Rz), '-b', 'DisplayName', 'Setpoint Rz');
    title('Rz Orientation');
    xlabel('Time (s)');
    ylabel('Rz Orientation (deg)');
    legend;
    grid on;
    grid minor;
    hold off;

    figure('Name', 'acceleration Z vs Time (s)', 'NumberTitle', 'off');
    hold on;

    Ts = mean(diff(time));
    ddz = filter([1,-2,1],Ts^2,z);
    plot(tsim(3:end) , azsim(3:end)/g, '-r', 'DisplayName', 'Simscape accZ');
    plot(time (3:end) ,ddz(3:end)/g, '-b', 'DisplayName', 'Setpoint accZ');
    title('Z axis acceleration');
    xlabel('Time (s)');
    ylabel('acceleration Z(g)');
    legend;
    grid on;
    grid minor;
    hold off;
    fprintf("Peak acceleration: Setpoint = %.3fg, Actual = %.3fg\n", max(ddz(3:end)/9.81), max(azsim(3:end)/9.81));

    
    figure('Name', 'AccZ (m/s^2) vs Theta (degrees)', 'NumberTitle', 'off');
    colors = ['r', 'g', 'b', 'c', 'm', 'y'];
    thetas = out.simout.Data(:, (2:4:22)-1);
    hold on;
    time_plots = gobjects(1, 6); 
    labels = cell(1, 6);
    for i = 1:6
        % Plot torque vs time for each motor
        time_plots(i) = plot(rad2deg(thetas(:, i)), azsim, 'Color', colors(i));
        labels{i} = ['Motor ' num2str(i)]; % Store labels in a cell array
    end
    
    % Add legend with motor labels
    legend(time_plots, labels, 'Location', 'northwest');
    
    xlabel('Theta (degrees)');
    ylabel('AccZ (m/s^2)');
    title('AccZ (m/s^2) vs Theta (degrees)');
    grid on;
    grid minor;
    hold off;
end