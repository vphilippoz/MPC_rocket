addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/10;
rocket = Rocket(Ts);

%Horizon lenth in seconds
H = 1.6;
nmpc = NMPC_Control(rocket, H);

%initial point set as origin
x0 = zeros(12,1);
%MPC with maximum roll angle of 15°
Tf = 30;
ref = @(t_, x_) rocket.MPC_ref(t_, Tf);

%MPC reference with maximum roll angle of 50°
roll_max = deg2rad(50);
ref = @(t_, x_) rocket.MPC_ref(t_, Tf, roll_max);

[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

%pelot
rocket.anim_rate = 2; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
