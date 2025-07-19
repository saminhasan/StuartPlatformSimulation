efficiency = 0.9;
g = 9.80665;
platform_mass =0.001; % kg
N = 9; % gear ratio
Jm = 12e-5; % motor inertia in motor frame ( kg . m^2)
Jmr = Jm*N^2; % motor inertia in robot frame ( kg . m^2)
cr = 1e-2;
cl = 1e-2;
density_cyl = (2 * (Jm * N^2)) / (pi * cr^4 * cl);
% rod_mass = 0.55; % kg
arm_mass  = 0.33; % kg
rod_mass = 1e-3; % kg
% arm_mass  = 1e-3; % kg
peak_torque = 45*efficiency/N; % mit motor peak torque in motor frame
rated_torque = 13*efficiency/N; % mit motor rated torque in motor frame
% 

Jr = (platform_mass / 6) * r^2; % robot equivalent inertia in robot frame
Je = (Jmr + Jr);
% disp(['Inertia Ratio : ', num2str(Jr/Jmr)]);
% disp(['Inertia Matched platform weight : ', num2str((Jmr/(r^2 )) * 6), 'Kg']);
f_trajectory = 3;
tau_0 = ((platform_mass / 6) * g * r) + (arm_mass * g * r / 2) + (rod_mass* g * r);
angular_spring_constant = ((((platform_mass / 6) * r^2) + (rod_mass * r^2) ...
    + (arm_mass * (r/2)^2) + Jmr) * (2 * pi * f_trajectory)^2);
angular_spring_offset = ((tau_0) / angular_spring_constant);
Ke = angular_spring_constant;
% % Control parameters
w_traj = f_trajectory * 2 * pi; % trajectory frequency
wn = w_traj * 10;
% wn = 2 * pi * 20;
zeta = sqrt(2); % damping ratio
P = wn^2 * Je - Ke; % proportional gain
D = 2 * zeta * wn * Je; % derivative gain
w_f = wn * 10; % filter frequency