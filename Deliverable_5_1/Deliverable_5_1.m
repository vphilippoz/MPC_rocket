addpath(fullfile('..', 'src'));
addpath('V:\Vincent\Documents\EPFL\MA1\Model predictive controll\casadi-windows-matlabR2016a-v3.5.5')

%% TODO 5.1: This file should produce all the plots for the deliverable
clear; close all; clc;

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

x0 = zeros(12,1);

H = 5; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);


% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Setup reference funtion
Tf = 30;
ref = @(t_, x_) rocket.MPC_ref(t_ , Tf);

rocket.mass = 1.783; % Manipulate mass for simulation
[T, X, U, Ref, Z_hat] = rocket.simulate_f_est_z(x0, Tf, mpc, ref, mpc_z, sys_z);
% [T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);


%pelot
% figure;
% plot(Z_hat(end,:))
% title('d_est')
% 
% figure;
% plot(Z_hat(end-1,:))
% title('z')


rocket.anim_rate = 30; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title




