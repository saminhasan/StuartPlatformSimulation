smoothStep = 0;
cycle = 0;
% rB = (((609.6 ) * sqrt(3) / 6) - 59.85) /1000
[r, n, rB, dB, rP, dP] = deal(0.1, 3.5878,0.1161, 0.106/2, 0.0716025403784439,  0.02);
fs = 1000;
dt = 1/fs;

efficiency = 0.9;
g = 9.80665;
torso_mass = 2.5; % kg
platform_mass= 10.0 - torso_mass; % kg
N = 9; % gear ratio
Jm = 12e-5; % motor inertia in motor frame ( kg . m^2)
Jmr = Jm*N^2; % motor inertia in robot frame ( kg . m^2)
cr = 1e-2;
cl = 1e-2;
density_cyl = (2 * (Jm * N^2)) / (pi * cr^4 * cl);
rod_mass = 0.55; % kg
arm_mass  = 0.33; % kg
m = platform_mass + 3*rod_mass;
Iplat = diag([0.00535189, 0.00535189, 0.0106204]);
J = Jmr + (arm_mass * (r/2)^2) + ((rod_mass/2) * (r)^2);
peak_torque = 45*efficiency/N; % mit motor peak torque in motor frame
rated_torque = 13*efficiency/N; % mit motor rated torque in motor frame


Jr = (platform_mass / 6) * r^2; % robot equivalent inertia in robot frame
Je = (Jmr + Jr);
% disp(['Inertia Ratio : ', num2str(Jr/Jmr)]);
% disp(['Inertia Matched platform weight : ', num2str((Jmr/(r^2 )) * 6), 'Kg']);
tau_0 = ((platform_mass / 6) * g * r) + (arm_mass * g * r / 2) + (rod_mass* g * r);

% % Control parameters
wn = 2 * pi * 15;
% zeta = 1/ sqrt(2); % damping ratio
zeta = 0.6; % damping ratio
P = wn^2 * Je; % proportional gain
D = 2 * zeta * wn * Je; % derivative gain
w_f = wn*5; % derivative filter frequency


riseTime = 0.1;
order = 20;
tauK = riseTime/ (gammaincinv(0.9, order) - gammaincinv(0.1, order));
if (smoothStep)
    riseDealy = calcDealy(order,tauK); %#ok<UNRCH>
else
    riseDealy = 0;
end
%fliplr(poly(ones(1, order) * -1) .* tau.^(0:order));  % (taus + 1)^25