addpath(fullfile('..', 'src'));
close all;clear;clc;
%% TODO: This file should produce all the plots for the deliverable
Ts = 1/10;
rocket = Rocket(Ts);

%Horizon lenth in seconds
H = 1.5;
nmpc = NMPC_Control(rocket, H);

%initial point set as origin
x0 = zeros(12,1);
%MPC with maximum roll angle of 15°
Tf = 30;

%Uncomment the next line to use the 15° maximum roll angle
% ref = @(t_, x_) rocket.MPC_ref(t_, Tf);

%MPC reference with maximum roll angle of 50°  COMMENT THE NEXT TWO LINES
%TO USE THE 15° maximum roll angle 
roll_max = deg2rad(50);
ref = @(t_, x_) rocket.MPC_ref(t_, Tf, roll_max);

[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

%pelot
rocket.anim_rate = 2; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
%% linear controller

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

%MPC reference with maximum roll angle of 50°
roll_max = deg2rad(50); 

ref = @(t_, x_) rocket.MPC_ref(t_, Tf, roll_max);

[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);
%% 
%plot
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title