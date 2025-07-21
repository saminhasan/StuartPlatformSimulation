function plotMotorAngles(motorAngles)
    % Create a figure for motor angles
    f1 = figure('Name', 'Motor Angles (degrees) vs Time (s)', 'NumberTitle', 'off');
    f1.Theme = "light";
    time = motorAngles(:,1);
    colors = ['r', 'g', 'b', 'm', 'c', 'k'];
    for i = 1:6
        subplot(3, 2, i); % 3 rows, 2 columns, i-th subplot
        angle_deg = rad2deg(motorAngles(:, i+1));
        plot(time, angle_deg, 'Color', colors(i));
        title(['Motor ' num2str(i)]);
        xlabel('Time (s)');
        ylabel('Motor Angles (degrees)');
        grid on; grid minor;
    end
end
