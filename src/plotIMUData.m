function plotIMUData(imu_data)
    % Load data into variables with meaningful names
    time = imu_data(:, 1);
    time = time - time(1);                % normalize to start at zero

    % Extract data components
    x_acc = imu_data(:, 2);               % X Linear Acceleration (m/s^2)
    y_acc = imu_data(:, 3);               % Y Linear Acceleration (m/s^2)
    z_acc = imu_data(:, 4);               % Z Linear Acceleration (m/s^2)
    x_ang_vel = imu_data(:, 5);           % Body rate p (deg/s)
    y_ang_vel = imu_data(:, 6);           % Body rate q (deg/s)
    z_ang_vel = imu_data(:, 7);           % Body rate r (deg/s)

    %% Figure 1: Linear Acceleration
    figAcc = figure('Name','IMU Linear Acceleration','NumberTitle','off');
    figAcc.Theme = "light";
    hold on;
    plot(time, x_acc, 'r', 'LineWidth', 1.2, 'DisplayName', '$\ddot{x}$');
    plot(time, y_acc, 'g', 'LineWidth', 1.2, 'DisplayName', '$\ddot{y}$');
    plot(time, z_acc, 'b', 'LineWidth', 1.2, 'DisplayName', '$\ddot{z}$');
    hold off;
    grid on; box on;
    title('Linear Acceleration', 'Interpreter','latex');
    xlabel('Time (s)', 'Interpreter','latex');
    ylabel('Acceleration $[\mathrm{m/s^2}]$', 'Interpreter','latex');
    ax1 = gca; ax1.TickLabelInterpreter = 'latex';
    leg1 = legend('show','Location','best'); set(leg1,'Interpreter','latex');

    %% Figure 2: Body Rates (p, q, r)
    fig2 = figure('Name','IMU Body Rates','NumberTitle','off');
    fig2.Theme = "light";
    hold on;
    plot(time, x_ang_vel, 'r', 'LineWidth', 1.2, 'DisplayName', '$p$');
    plot(time, y_ang_vel, 'g', 'LineWidth', 1.2, 'DisplayName', '$q$');
    plot(time, z_ang_vel, 'b', 'LineWidth', 1.2, 'DisplayName', '$r$');
    hold off;
    grid on; box on;
    title('Body Rates', 'Interpreter','latex');
    xlabel('Time (s)', 'Interpreter','latex');
    ylabel('Angular velocity $[\mathrm{deg/s}]$', 'Interpreter','latex'); % change to rad/s if needed
    ax2 = gca; ax2.TickLabelInterpreter = 'latex';
    leg2 = legend('show','Location','best'); set(leg2,'Interpreter','latex');
end