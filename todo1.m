% TODO 1.1
clc; clear; close all;

Ts = 1/20;
rocket = Rocket(Ts);

% Command inputs
d1 =    0*pi/180; % [-15°, +15°]
d2 =    0*pi/180; % [-15°, +15°]
Pavg =  56.5; % [20%, 80%] already in %
Pdiff = 0; % [-20%, +20%]
u = [d1, d2, Pavg, Pdiff]';

[b_F, b_M] = rocket.getForceAndMomentFromThrust(u);

% Initial pose (in SI units)
w =   [0 0 0]; % angular velocities about the body axes
phi = [0 0 0]; % angular position of the body frame with respect to the world frame
v =   [0 0 0]; % linear velocities expressed in the world frame
p =   [0 0 0]; % linear position expressed in the world frame

x0 = [w, phi, v, p]'; 
x_dot = rocket.f(x0, u);

% TODO 1.2
rocket = Rocket(Ts);
Tf = 2.0; % Time to simulate for

[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 0.1;
rocket.vis(T, X, U); % Trajectory visualization at 1.0x real−time