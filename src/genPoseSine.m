function trajectory= genPoseSine(n)

    f_trajectory = 1.5;
    tf = n / f_trajectory;
    dt = 1e-3;
    time = (0:dt:tf)';

    x = 0.05  * sin(2 * pi * f_trajectory * time)*0; % x (horizontal component, front back)
    y = 0.05  * sin(2 * pi * f_trajectory * time)*0; % y (horizontal component, left side right side)
    z = 0.04 * sin(2 * pi * 2 * f_trajectory * time); % z (vertical component, up down)
    Rx = deg2rad(3.0)  * sin(2 * pi * f_trajectory * time); % roll
    Ry = deg2rad(2.5)  * sin(2 * pi * 2 * f_trajectory * time); % pitch
    Rz = deg2rad(15.0) * sin(2 * pi * f_trajectory * time); % yaw

    trajectory = [time, x, y, z, Rx, Ry, Rz];
end

