function trajectory = genPoseMix(n)
    trajectory = genTrajectory('cam', n);
    g=9.80665;
    gm=1.5*g/2;    % muscle created g, leave at this value
    p=3/8;%3/8 = 0.375 %.33;         % stride period: between 0.33 and 0.44
                   % changes the maximum displacement:
                   %    p=0.33 --> -4cm to +2cm
                   %    p=0.40 --> -6cm to +2cm
    amax = 6*g;    % maximum accelertaion, little impact on displacement or velocity
    time = trajectory(:,1) ;
    % internal model parameters, do not touch
    o = (amax - 4*g + 4*gm + sqrt(amax^2 + 8*amax*g - 8*amax*gm))/(4*(g - gm));
    j1=p*(g*o - gm*o + g - gm)/(p*(o + 2)/(2*(o + 1)))^(o + 1);
    tE=p*(o + 2)/(2*(o + 1));
    t0=p/3;
    phi0=2*pi/p*t0;
    t_mod = mod(time + 0.0640,p); % to match with cam trajectory
    trajectory(:,4) = (-g + gm)/2 * t_mod.^2 - gm * (p/2/pi)^2 * cos(2*pi/p*t_mod+phi0) + j1/(o+1)/(o+2) * min(t_mod,tE).^(o+2) + j1/(o+1)*((t_mod>tE).*tE).^(o+1).*(t_mod-tE);
end
