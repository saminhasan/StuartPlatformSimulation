function [trajectory, tf, ts] = genPose(n)
    ts= 1e-3;
    % ****************
    % Model parameters
    % ****************
    g=9.81;
    gm=1.5*g/2;    % muscle created g, leave at this value
    p=.33;         % stride period: between 0.33 and 0.44
                   % changes the maximum displacement:
                   %    p=0.33 --> -4cm to +2cm
                   %    p=0.40 --> -6cm to +2cm
    amax = 6*g;    % maximum accelertaion, little impact on displacement or velocity
    time = (0:ts:p*n)' ;
    tf = time(end);

    % internal model parameters, do not touch
    o = (amax - 4*g + 4*gm + sqrt(amax^2 + 8*amax*g - 8*amax*gm))/(4*(g - gm));
    j1=p*(g*o - gm*o + g - gm)/(p*(o + 2)/(2*(o + 1)))^(o + 1);
    dt=p*(o + 2)/(2*(o + 1));
    t0=p/3;
    phi0=2*pi/p*t0;
    v0=gm*p/2/pi*sin(2*pi/p*t0); %#ok<NASGU>
    % time vector needs to restart from zero after each period
    t_mod=mod(time-0.125,p);
    a = (-g + gm)          + gm              * cos(2*pi/p*t_mod+phi0) + j1             * (t_mod.*(t_mod<dt)).^o; %#ok<NASGU>
    v = (-g + gm)   * t_mod    + gm * (p/2/pi)   * sin(2*pi/p*t_mod+phi0) + j1/(o+1)       * min(t_mod,dt).^(o+1); %#ok<NASGU>
    s = (-g + gm)/2 * t_mod.^2 - gm * (p/2/pi)^2 * cos(2*pi/p*t_mod+phi0) + j1/(o+1)/(o+2) * min(t_mod,dt).^(o+2) + j1/(o+1)*((t_mod>dt).*dt).^(o+1).*(t_mod-dt);
    f_trajectory = 1/p;
    z = (s - mean(s));
    x = 0.05  * sin(2 * pi * f_trajectory * time)*0; % z (horizontal component, front back)
    y = 0.05  * sin(2 * pi * f_trajectory * time)*0; % x (horizontal component, left right)
    
    Rz = deg2rad(15.0)  *  sin(2 * pi * f_trajectory/2 * time + phi0 -pi/2)*0;
    Rx = deg2rad(3.0)  *  sin(2 * pi * f_trajectory/2 * time+ phi0 -pi/2)*0;
    Ry = deg2rad(2.5) * sin(2 * pi *  f_trajectory * time+ phi0 -pi/2)*0;
    trajectory = [time, x, y, z, Rx, Ry, Rz];
end