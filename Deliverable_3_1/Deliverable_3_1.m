%% TODO: This file should produce all the plots for the deliverable
clc; clear; close all;
addpath(fullfile('..','src'));
%addpath('V:\Vincent\Documents\EPFL\MA1\Model predictive controll\casadi-windows-matlabR2016a-v3.5.5')

% Initial pose (in SI units)
x0 = [0, 0, 0, 5].'; % for system x: ωy , β, vx , x
y0 = [0, 0, 0, 5].'; % for system y: ωx , α, vy , y
z0 = [0, 5].'; % for system z: vz, z
roll0 = [0, deg2rad(45)].'; % for system roll: ωz, γ

% Create rocket
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
H = 4; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Get control input
ux = mpc_x.get_u(x0);
uy = mpc_y.get_u(y0);
uz = mpc_z.get_u(z0);
uroll = mpc_roll.get_u(roll0);

% Simulate
Tf = 10; % [s]

%   x
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%   y
[T, X_sub, U_sub] = rocket.simulate(sys_y, y0, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%   z
[T, X_sub, U_sub] = rocket.simulate(sys_z, z0, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);

%   roll
[T, X_sub, U_sub] = rocket.simulate(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);