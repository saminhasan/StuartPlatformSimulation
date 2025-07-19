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

%% 3) Rotation + angular velocity
Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
R  = Rx*Ry*Rz;

Rdot = diff(R,phi)*phidot ...
     + diff(R,theta)*thetadot ...
     + diff(R,psi)*psidot;
S     = simplify(R.'*Rdot);
omega = [S(3,2); S(1,3); S(2,1) ];
E_sym = jacobian( omega, [phidot; thetadot; psidot] )
% t        = X(1:3);
% P_trans  = (R*P.' + t).';    % 6×3
% l        = P_trans - B;      % 6×3
% ek       = 2*r*l(:,3);       % 6×1
% fk       = 2*r*(cos(beta.').*l(:,1) + sin(beta.').*l(:,2));
% L_sq     = sum(l.^2,2);      % 6×1
% theta_k  = asin((L_sq - ((n^2 - 1)*r^2)) ./ sqrt(ek.^2 + fk.^2)) - atan2(fk,ek);   % 6×1
% theta_k
% JX = jacobian(theta_k, X);
% JX
% theta_kdot = JX * Xdot;
% theta_kdot
% T_trans = 1/2*m*(xdot^2 + ydot^2 + zdot^2);
% Iplat   = diag([Ix, Iy, Iz]);
% T_rot   = 1/2*(omega.'*Iplat*omega);
% T_mot   = 1/2*J*sum(theta_kdot.^2);
% T = T_trans + T_rot + T_mot;
% V = m*g*z;
% L = T - V;
% L
%---------------------------------------------------------
% % 4.1) Make placeholders for the 6 θ̇ₖ’s
% syms th1d th2d th3d th4d th5d th6d real
% theta_kdot_sym = [th1d; th2d; th3d; th4d; th5d; th6d];
% 
% % 4.2) Substitute into L so it “sees” those as independent vars
% L_tmp = subs(L, theta_kdot, theta_kdot_sym);
% L_tmp
% % 4.3) Compute ∂L/∂θ̇ₖ  (6×1)
% dL_dtheta_kdot_tmp = jacobian(L_tmp, theta_kdot_sym).';
% dL_dtheta_kdot_tmp
% % 4.4) Put the real θ̇ₖ back in
% dL_dtheta_kdot = subs(dL_dtheta_kdot_tmp, theta_kdot_sym, theta_kdot);
% dL_dtheta_kdot
% % 4.5) Time‐derivative d/dt(∂L/∂θ̇ₖ)
% d2L_dtheta_kdot_dt = (jacobian(dL_dtheta_kdot, [X; Xdot]) * [Xdot; Xddot]);
% d2L_dtheta_kdot_dt
% % 4.6) Euler–Lagrange torque (since ∂L/∂θₖ=0)
% tau = d2L_dtheta_kdot_dt