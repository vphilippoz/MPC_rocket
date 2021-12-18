%% TODO 3.1
clc; clear; close all;
addpath(fullfile('src'));
addpath(fullfile('Deliverable_3_1'));
addpath('V:\Vincent\Documents\EPFL\MA1\Model predictive controll\casadi-windows-matlabR2016a-v3.5.5')

% Initial pose (in SI units)
% w =   [0 0 0]; % angular velocities about the body axes
% phi = [0 0 0]; % angular position of the body frame with respect to the world frame
% v =   [0 0 0]; % linear velocities expressed in the world frame
% p =   [0 0 0]; % linear position expressed in the world frame
% 
% x0 = [w, phi, v, p]'; 
x0 = [0.2, deg2rad(2), 0, 0].'; % for system x: ωy , β, vx , x.

% Create rocket
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
H = 1; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
% Get control input
ux = mpc_x.get_u(x0) 

% Simulate
Tf = 1; % [s]
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

