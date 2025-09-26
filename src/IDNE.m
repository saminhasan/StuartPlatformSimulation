clc; close all; clear all; %#ok<CLALL>
% inverse dynamic newton euler.
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
mode = 'imu';
trajectory = genTrajectory(mode, 5);
jointAngles = sp.move(trajectory);
time = trajectory(:,1);
tf = time(end);
%--------------------------------------------------------------------------

theta_k = jointAngles(:,2:7); theta_kdot = filter([1, 0 , -1], dt*2, theta_k);theta_kddot = filter([1, -2 , 1], dt^2, theta_k);
X = trajectory(:,2:7);Xdot = filter([1, 0 , -1], dt*2, X);Xddot = filter([1, -2 , 1], dt^2, X);
phi     = X(:,4);theta = X(:,5); psi = X(:,6);
phidot  = Xdot(:,4);thetadot= Xdot(:,5);psidot  = Xdot(:,6);
phiddot  = Xddot(:,4);thetaddot= Xddot(:,5);psiddot  = Xddot(:,6);

%--- angular velocity (world frame) ---------------------------------------

omega = [
    phidot - psidot.*sin(theta), ...
    psidot.*sin(phi).*cos(theta) + thetadot.*cos(phi), ...
    psidot.*cos(phi).*cos(theta) - thetadot.*sin(phi)
];

%--- its time‐derivative --------------------------------------------------
omega_dot = [
    phiddot - psiddot.*sin(theta) - psidot.*thetadot.*cos(theta), ...
    -phidot.*thetadot.*sin(phi) + psiddot.*sin(phi).*cos(theta) + psidot.*(phidot.*cos(phi).*cos(theta) - thetadot.*sin(phi).*sin(theta)) + thetaddot.*cos(phi), ...
    -phidot.*thetadot.*cos(phi) + psiddot.*cos(phi).*cos(theta) - psidot.*(phidot.*sin(phi).*cos(theta) + thetadot.*sin(theta).*cos(phi)) - thetaddot.*sin(phi)
];
%--------------------------------------------------------------------------

taus = zeros(size(X));
for idx = 1 : size(taus,1)
  % linear force:
  F_i       = m * Xddot(idx,1:3)';            % 3×1
  M_body        = Iplat*omega_dot(idx,:)' + cross(omega(idx,:)', Iplat*omega(idx,:)');  % 3×1
  W         = [ F_i + [0;0;m*g] ; M_body ];              % 6×1
  JXi       = fnJacobianSP(sp.B, sp.P, sp.betaB, sp.r, sp.n, X(idx,:));      % 6×6
  tau_plat  = (JXi') \ W;                             % 6×1
  taus(idx,:)  = (tau_plat + J*theta_kddot(idx,:)')' ;
end


out = sim("SP.slx");

sim_time = out.simout.Time;
sim_dt = mean(diff(sim_time));
taus_load = out.simout.Data(:, (2:4:22) + 2)/N;
taus = taus/ N;
% 
close all;
colors = 'rgbcmyrgbcmy';
for k = 1:6
    figure(k+10);
    hold on;
    plot( time(3:end) + riseDealy, taus(3:end, k), 'Color',colors(k), 'LineStyle','-','DisplayName',sprintf('Motor %d (Calculated)', k) );
    plot( sim_time,taus_load(:, k),'Color',colors(k+1),'LineStyle','--','DisplayName', sprintf('Motor %d (Simulated)', k) );
    xlabel('Time [s]');
    ylabel('Torque [Nm]');
    title('Servo Torques (with Rotational Inertia)');
    grid on;

    % constant torque limits
    yline( peak_torque,  '-r', 'T-motor-peak','HandleVisibility','off','LabelVerticalAlignment','top');
    yline( rated_torque, '-g', 'T-motor-rated','HandleVisibility','off','LabelVerticalAlignment','top');
    yline(-rated_torque, '-g', 'T-motor-rated','HandleVisibility','off','LabelVerticalAlignment','bottom');
    yline(-peak_torque,  '-r', 'T-motor-peak','HandleVisibility','off','LabelVerticalAlignment','bottom');
    legend('Location', 'northeast');
end
