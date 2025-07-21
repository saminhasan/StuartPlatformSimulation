function plotTrajectory(trajectory)
    time = trajectory(:,1);
    x = trajectory(:,2);
    y = trajectory(:,3);
    z = trajectory(:,4);
    
    Rx = trajectory(:,5);
    Ry = trajectory(:,6);
    Rz = trajectory(:,7);

    figure('Name', 'pose vs Time (s)', 'NumberTitle', 'off');
    title("Pose vs Time");
    subplot(3,2,1);
    plot(time, x, Color='r', DisplayName='X');
    title('X Position');
    xlabel('Time (s)');
    ylabel('X (m)');
    legend;
    grid on;
    grid minor;
    
    subplot(3,2,2);
    plot(time, y, Color='g', DisplayName='Y');
    title('Y Position');
    xlabel('Time (s)');
    ylabel('Y (m)');
    legend;
    grid on;
    grid minor;
    
    subplot(3,2,3);
    plot(time, z, Color='b', DisplayName='Z');
    title('Z Position');
    xlabel('Time (s)');
    ylabel('Z (m)');
    legend;
    grid on;
    grid minor;
    
    subplot(3,2,4);
    plot(time, rad2deg(Rx), Color='r', DisplayName='roll');
    title('Rx Orientation');
    xlabel('Time (s)');
    ylabel('Roll (deg)');
    legend;
    grid on;
    grid minor;
    
    subplot(3,2,5);
    plot(time, rad2deg(Ry), Color='g', DisplayName='pitch');
    title('Ry Orientation');
    xlabel('Time (s)');
    ylabel('Pitch (deg)');
    legend;
    grid on;
    grid minor;
    
    subplot(3,2,6);
    plot(time, rad2deg(Rz), Color='g', DisplayName='yaw');
    title('Rz Orientation');
    xlabel('Time (s)');
    ylabel('Yaw (deg)');
    legend;
    grid on;
    grid minor;
end