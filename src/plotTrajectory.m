function plotTrajectory(trajectory)
    time = trajectory(:,1);
    z = trajectory(:,4);

    Rx = trajectory(:,5); % roll (rad)
    Ry = trajectory(:,6); % pitch (rad)
    Rz = trajectory(:,7); % yaw (rad)

    f_pose = figure('Name', 'Pose vs Time', 'NumberTitle', 'off');
    f_pose.Theme = "light";

    % Z Position
    subplot(2,2,1);
    plot(time, z, 'b', 'LineWidth', 1.2, 'DisplayName', '$z$');
    title('Vertical Position', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('Position $[\mathrm{m}]$', 'Interpreter', 'latex');
    ax1 = gca; ax1.TickLabelInterpreter = 'latex';
    leg1 = legend('show', 'Location', 'best'); set(leg1, 'Interpreter', 'latex');
    grid on; grid minor;

    % Roll (phi)
    subplot(2,2,2);
    plot(time, rad2deg(Rx), 'r', 'LineWidth', 1.2, 'DisplayName', '$\phi$');
    title('Roll $\phi$', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('Angle(degrees)', 'Interpreter', 'latex');
    ax2 = gca; ax2.TickLabelInterpreter = 'latex';
    leg2 = legend('show', 'Location', 'best'); set(leg2, 'Interpreter', 'latex');
    grid on; grid minor;

    % Pitch (theta)
    subplot(2,2,3);
    plot(time, rad2deg(Ry), 'g', 'LineWidth', 1.2, 'DisplayName', '$\theta$');
    title('Pitch $\theta$', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('Angle(degrees)', 'Interpreter', 'latex');
    ax3 = gca; ax3.TickLabelInterpreter = 'latex';
    leg3 = legend('show', 'Location', 'best'); set(leg3, 'Interpreter', 'latex');
    grid on; grid minor;

    % Yaw (psi)
    subplot(2,2,4);
    plot(time, rad2deg(Rz), 'm', 'LineWidth', 1.2, 'DisplayName', '$\psi$');
    title('Yaw $\psi$', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('Angle(degrees)', 'Interpreter', 'latex');
    ax4 = gca; ax4.TickLabelInterpreter = 'latex';
    leg4 = legend('show', 'Location', 'best'); set(leg4, 'Interpreter', 'latex');
    grid on; grid minor;
end


% function plotTrajectory(trajectory)
%     time = trajectory(:,1);
%     x = trajectory(:,2);
%     y = trajectory(:,3);
%     z = trajectory(:,4);
% 
%     Rx = trajectory(:,5);
%     Ry = trajectory(:,6);
%     Rz = trajectory(:,7);
% 
%     f_pose = figure('Name', 'pose vs Time (s)', 'NumberTitle', 'off');
%     f_pose.Theme = "light";
%     title("Pose vs Time");
%     subplot(3,2,1);
%     plot(time, x, Color='r', DisplayName='X');
%     title('X Position');
%     xlabel('Time (s)');
%     ylabel('X (m)');
%     legend;
%     grid on;
%     grid minor;
% 
%     subplot(3,2,2);
%     plot(time, y, Color='g', DisplayName='Y');
%     title('Y Position');
%     xlabel('Time (s)');
%     ylabel('Y (m)');
%     legend;
%     grid on;
%     grid minor;
% 
%     subplot(3,2,3);
%     plot(time, z, Color='b', DisplayName='Z');
%     title('Z Position');
%     xlabel('Time (s)');
%     ylabel('Z (m)');
%     legend;
%     grid on;
%     grid minor;
% 
%     subplot(3,2,4);
%     plot(time, rad2deg(Rx), Color='r', DisplayName='roll');
%     title('Rx Orientation');
%     xlabel('Time (s)');
%     ylabel('Roll (deg)');
%     legend;
%     grid on;
%     grid minor;
% 
%     subplot(3,2,5);
%     plot(time, rad2deg(Ry), Color='g', DisplayName='pitch');
%     title('Ry Orientation');
%     xlabel('Time (s)');
%     ylabel('Pitch (deg)');
%     legend;
%     grid on;
%     grid minor;
% 
%     subplot(3,2,6);
%     plot(time, rad2deg(Rz), Color='g', DisplayName='yaw');
%     title('Rz Orientation');
%     xlabel('Time (s)');
%     ylabel('Yaw (deg)');
%     legend;
%     grid on;
%     grid minor;
% end