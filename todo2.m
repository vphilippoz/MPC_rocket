% TODO 2.1
clc; clear; close all;
addpath(fullfile('..', 'src'));

Ts = 1/20; % [s] sampling time

rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point

% TODO 2.2
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)