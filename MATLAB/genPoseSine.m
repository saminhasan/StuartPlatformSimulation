function [trajectory, tf, ts] = genPoseSine(n)
    f_trajectory = 1/0.75;
    tf = n / f_trajectory;
    ts = 1e-3;
    time = (0:ts:tf)';
    % warpedTime = warpTime(time,1/f_trajectory);
    warpedTime = time;
    x = 0.05  * sin(2 * pi * f_trajectory * warpedTime)*0; % x (horizontal component, front back)
    y = 0.05  * sin(2 * pi * f_trajectory * warpedTime)*0; % y (horizontal component, left side right side)
    z = 0.04 * sin(2 * pi * 2 * f_trajectory * warpedTime); %  (vertical component, up down)

    Rx = deg2rad(3.0)  *  sin(2 * pi * f_trajectory * warpedTime);
    Ry = deg2rad(2.5) * sin(2 * pi * 2 * f_trajectory * warpedTime);
    Rz = deg2rad(15.0)  *  sin(2 * pi * f_trajectory * warpedTime);

    trajectory = [time, x, y, z, Rx, Ry, Rz];

end

