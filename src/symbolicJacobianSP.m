%% symbolicJacobianSP.m
clc; clear; close all;
%% Declare symbolic vectors and scalars
syms r n real
syms P [6 3] real      % P = [Px; Py; Pz]
syms B [6 3] real      % B = [Bx; By; Bz]
syms beta [1 6] real
syms x y z phi theta psi real
X = [x, y, z, phi, theta, psi];
T = [x ;y; z];
%% Rotation matrices
Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi), cos(phi)];
Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
R  = Rx*Ry*Rz;
t = [x; y; z];
P = (R * P.' + t).'; % transformed points
l = P - B; 
ek = 2*r*l(:,3);
fk = 2*r*(cos(beta.').*l(:,1) + sin(beta.').*l(:,2));
L_sq = sum(l.^2,2);
theta_k = asin((L_sq - ((n^2 - 1)*r^2))./sqrt(ek.^2 + fk.^2)) - atan2(fk,ek);
JX = jacobian(theta_k, X);
% J_theta = jacobian(X, angles);
% matlabFunction(J_sym, 'File', 'numericJacobianSP');