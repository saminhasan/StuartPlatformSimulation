clc; close all; clear all; %#ok<CLALL>
addpath('quaternion_library');
% rB = (((609.6 ) * sqrt(3) / 6) - 59.85) /1000
[r, n, rB, dB, rP, dP] = deal(0.1, 3.5878,0.1161, 0.106/2, 0.0716025403784439,  0.02);
params

sp = StuartPlatform(r, n, rB, dB, rP, dP);
mode = 'syn';
[trajectory, tf, ts] = genTrajectory(mode, 5);
motorAngles = zeros(size(trajectory));
for idx = 1:size(trajectory,1)
    motorAngles(idx,:) = [trajectory(idx,1), sp.move(trajectory(idx,2:7))'];
end
time = trajectory(:,1);
theta_k = motorAngles(:,2:7); theta_kdot = filter([1, 0 , -1], ts*2, theta_k);theta_kddot = filter([1, -2 , 1], ts^2, theta_k);
X = trajectory(:,2:7);Xdot = filter([1, 0 , -1], ts*2, X);Xddot = filter([1, -2 , 1], ts^2, X);
m = platform_mass;
% Iplat = diag([0.00535189, 0.00535189, 0.0106204]);1.07038e-06, 1.07038e-06, 2.12409e-06
Iplat = diag([1.07038e-06, 1.07038e-06, 2.12409e-06]);%1.07038e-06, 1.07038e-06, 2.12409e-06
J = Jmr + (arm_mass*(sp.r^2)/4);
phi     = X(:,4);theta = X(:,5); psi = X(:,6);
phidot  = Xdot(:,4);thetadot= Xdot(:,5);psidot  = Xdot(:,6);
phiddot  = Xddot(:,4);thetaddot= Xddot(:,5);psiddot  = Xddot(:,6);
omega = [ ...
  thetadot.*sin(psi) + phidot.*cos(psi).*cos(theta),    ...
  thetadot.*cos(psi) - phidot.*sin(psi).*cos(theta),    ...
  psidot + phidot.*sin(theta)                          ...
];   % (N×3)
omega_dot = zeros(size(omega));
omega_dot(:,1) = thetaddot.*sin(psi) ...
             + thetadot.*psidot.*cos(psi) ...
             + phiddot.*cos(psi).*cos(theta) ...
             - phidot.*psidot.*sin(psi).*cos(theta) ...
             - phidot.*thetadot.*cos(psi).*sin(theta);

omega_dot(:,2) = thetaddot.*cos(psi) ...
             - thetadot.*psidot.*sin(psi) ...
             - phiddot.*sin(psi).*cos(theta) ...
             - phidot.*psidot.*cos(psi).*cos(theta) ...
             + phidot.*thetadot.*sin(psi).*sin(theta);

omega_dot(:,3) = psiddot ...
             + phiddot.*sin(theta) ...
             + phidot.*thetadot.*cos(theta);   % (N×3)


taus = zeros(size(X));
for idx = 1 : size(taus,1)
  % linear force:
  F_i       = m * Xddot(idx,1:3).';            % 3×1
  aP = alphaP(phidot(idx),phiddot(idx),psidot(idx),psi(idx),psiddot(idx),theta(idx),thetadot(idx),thetaddot(idx));
  % full rigid‑body moment:
  M_body        = Iplat*aP + cross(omega(idx,:).', Iplat*omega(idx,:).');  % 3×1
  E_i =   [ cos(psi(idx))*cos(theta(idx)),  sin(psi(idx)),         0;
           -cos(theta(idx))*sin(psi(idx)),  cos(psi(idx)),         0;
            sin(theta(idx)),                 0,          1];
  Q_euler= E_i' * M_body;                           % generalized moments
  % wrench = [force; moment] including gravity:
  W         = [ F_i + [0;0;m*g] ; Q_euler ];              % 6×1
  JXi       = numericJacobianSP(sp.B, sp.P, sp.betaB, sp.r, sp.n, X(idx,:));      % 6×6
  tau_plat  = (JXi.') \ W;                             % 6×1
  taus(idx,:)  = (tau_plat + J*theta_kddot(idx,:).').' ;
end
if any(imag(taus(:)) ~= 0)
    disp('taus has imaginary values');
else
    disp('taus is purely real');
end

out = sim("SP.slx");

sim_time = out.simout.Time;
taus_load = out.simout.Data(:, (2:4:22) + 2);
% 
close all;
colors = 'rgbcmyrgbcmy';
for k = 1:6
    figure(k+10);
    hold on;
    plot( time(100:end), taus(100:end, k), 'Color',colors(k), 'LineStyle','-','DisplayName',sprintf('Motor %d (Calculated)', k) );
    plot( sim_time(100:end),taus_load(100:end, k),'Color',colors(k+1),'LineStyle','--','DisplayName', sprintf('Motor %d (Simulated)', k) );
    xlabel('Time [s]');
    ylabel('Torque [Nm]');
    title('Servo Torques (with Rotational Inertia)');
    grid on;

    % constant torque limits
    yline( peak_torque*N,  '-r', 'T-motor-peak','HandleVisibility','off','LabelVerticalAlignment','top');
    yline( rated_torque*N, '-g', 'T-motor-rated','HandleVisibility','off','LabelVerticalAlignment','top');
    yline(-rated_torque*N, '-g', 'T-motor-rated','HandleVisibility','off','LabelVerticalAlignment','bottom');
    yline(-peak_torque*N,  '-r', 'T-motor-peak','HandleVisibility','off','LabelVerticalAlignment','bottom');
    legend('Location', 'northeast');
end