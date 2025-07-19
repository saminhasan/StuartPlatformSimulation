clc;  close all; clear all; %#ok<CLALL>

syms r n J m Ix Iy Iz g real
syms P [6 3] real
syms B [6 3] real
syms beta [1 6] real
syms x y z phi theta psi real
syms xdot ydot zdot phidot thetadot psidot real
syms xddot yddot zddot phiddot thetaddot psiddot real

X = [x, y, z, phi, theta, psi]';
Xdot = [xdot, ydot, zdot, phidot, thetadot, psidot]';
Xddot = [xddot, yddot, zddot, phiddot, thetaddot, psiddot]';

Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
R  = Rx*Ry*Rz;

Rdot = diff(R,phi)*phidot ...
     + diff(R,theta)*thetadot ...
     + diff(R,psi)*psidot;
S     = simplify(R.'*Rdot);
omega = [S(3,2); S(1,3); S(2,1) ];

t        = X(1:3);
P_trans  = (R*P.' + t).';    % 6×3
l        = P_trans - B;      % 6×3
ek       = 2*r*l(:,3);       % 6×1
fk       = 2*r*(cos(beta.').*l(:,1) + sin(beta.').*l(:,2));
L_sq     = sum(l.^2,2);      % 6×1
theta_k  = asin((L_sq - ((n^2 - 1)*r^2)) ./ sqrt(ek.^2 + fk.^2)) - atan2(fk,ek);   % 6×1
theta_k
% 1) build the platform‐wrench mass matrix
Mp = diag([m, m, m, Ix, Iy, Iz]);    % 6×6

% 2) gravitational wrench (force/torque on CoM + zero rotation)
Wg = [0; 0; m*g; 0; 0; 0];            % 6×1

% 3) inertial wrench from acceleration
Wi = Mp * Xddot;                     % 6×1

% 4) total wrench on platform
W = Wi + Wg;                         % 6×1
W
% 5) Jacobian dθ/dX
JX = jacobian(theta_k, X);           % 6×6
JX
% 6) actuator torques via virtual work: JX.' * τ = W  ⇒  τ = (JX.') \ W
tau = (JX.') \ W;  