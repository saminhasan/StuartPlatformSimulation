clc;  close all; clear all; %#ok<CLALL>

syms r n J m Ix Iy Iz g real
syms P [6 3] real         % platform‐frame attachment points
syms B [6 3] real         % base‐frame anchors
syms beta [1 6] real      % leg‐base azimuths
syms x y z phi theta psi real
syms xdot ydot zdot phidot thetadot psidot real

%% 3) Rotation + angular velocity
Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
R  = Rx*Ry*Rz;

Rdot = diff(R,phi)*phidot ...
     + diff(R,theta)*thetadot ...
     + diff(R,psi)*psidot;
S     = simplify(R.'*Rdot);
omega = [ S(3,2); S(1,3); S(2,1) ];
X = [x, y, z, phi, theta, psi]';
Xdot = [xdot; ydot; zdot; phidot; thetadot; psidot];
%% 4) Leg kinematics → theta_k(q)
t        = X(1:3);
P_trans  = (R*P.' + t).';    % 6×3
l        = P_trans - B;      % 6×3
ek       = 2*r*l(:,3);       % 6×1
fk       = 2*r*(cos(beta.').*l(:,1) + sin(beta.').*l(:,2));
L_sq     = sum(l.^2,2);      % 6×1
theta_k  = asin((L_sq - ((n^2 - 1)*r^2)) ./ sqrt(ek.^2 + fk.^2)) ...
           - atan2(fk,ek);   % 6×1
JX = jacobian(theta_k, X);
% 
theta_kdot = JX * Xdot;
%% 6) Energies & Lagrangian
T_trans = 1/2*m*(xdot^2 + ydot^2 + zdot^2);
Iplat   = diag([Ix, Iy, Iz]);
T_rot   = 1/2*(omega.'*Iplat*omega);
T_mot   = 1/2*J*sum(theta_kdot.^2);
T = T_trans + T_rot + T_mot;
V = m*g*z;
L = T - V;

%% (a) Declare your platform accelerations
syms xddot yddot zddot phiddot thetaddot psiddot real
Xddot = [xddot; yddot; zddot; phiddot; thetaddot; psiddot];

%% (b) Compute d/dt(∂L/∂X˙) via chain-rule
dLdXd = jacobian(L, Xdot).';                              % 6×1
d_dt_dLdXd = jacobian(dLdXd, X)*Xdot ...
           + jacobian(dLdXd, Xdot)*Xddot;                % 6×1

%% (c) Compute ∂L/∂X
dLdX   = jacobian(L, X).';                               % 6×1

%% (d) Generalized forces in X-space
Qx     = d_dt_dLdXd - dLdX;                  % 6×1

%% (e) Project onto joint actuators θₖ via virtual work:
%      Qx = JXᵀ * tau  ⇒   tau = (JXᵀ)\Qx
tau_sym = JX.' \ Qx;             