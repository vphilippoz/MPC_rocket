function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

%we start by defining the cost function to be minimised

%index of all the subsystems x variables

%cost of the various subsystems (cost =x_subsystem'* Q_subsystem* x_subsystem
Q = 50*eye(nx);

%terminal cost based on the lineraization of the system
[xs, us] = rocket.trim();
sys = rocket.linearize(xs,us);
sys_d = c2d(sys, rocket.Ts);


Rx = 0.5;       %cost of the inputs
Ry = 0.5;
Rz = 1;
Rroll = 0.5;
R = diag([Rx Ry Rz Rroll]);

%computing the LQR cost as a terminal set cost
[~,Qf,~] = dlqr(sys_d.A, sys_d.B, Q, R);

%setup the constraints 
%state constraints F*x <= f
F = [0 0 0 0 1 0 0 0 0 0 0 0;   %constraints on alpha
     0 0 0 0 -1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;   %constraints on beta
     0 0 0 0 0 -1 0 0 0 0 0 0];

f = [deg2rad(85); deg2rad(85); deg2rad(85); deg2rad(85)];

%input constraints M*u <= m
M =[1 0 0 0;    %d1 constraints
    -1 0 0 0;
    0 1 0 0;    %d2 constraints
    0 -1 0 0;
    0 0 1 0;    %d3 constraints
    0 0 -1 0;
    0 0 0 1;    %d4 constraints
    0 0 0 -1];

m = [deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15);
     80-56.667; -50+56.667; 20; 20];

%now we can implement the cost, we assume that the at the steady state, the
%rocket velocities are 0 and (since x_dot = 0 at steady state => all
%rotational and displacement speeds are 0) We also assume that the rocket
%is aligned with the z axis (alpha = beta = 0)

%target state
x_target = [0 0 0 0 0 ref_sym(4) 0 0 0 ref_sym(1) ref_sym(2) ref_sym(3)]';

% 
%compute the next state using a RK4 integration function
k1 = rocket.f(X_sym(:,1)-x_target, U_sym(:,1));
k2 = rocket.f(X_sym(:,1)-x_target+rocket.Ts/2*k1, U_sym(:,1));
k3 = rocket.f(X_sym(:,1)-x_target+rocket.Ts/2*k2, U_sym(:,1));
k4 = rocket.f(X_sym(:,1)-x_target+rocket.Ts*k3, U_sym(:,1));
   
obj = U_sym(:,1)'*R*U_sym(:,1);
opti.subject_to(M*U_sym(:,1) <= m);
opti.subject_to(X_sym(:,2)-x_target == X_sym(:,1)-x_target + rocket.Ts/6*(k1+2*k2+2*k3+k4));



for i = 2:N-1
    %RK4 integration
    k1 = rocket.f(X_sym(:,i)-x_target, U_sym(:,i));
    k2 = rocket.f(X_sym(:,i)-x_target+rocket.Ts/2*k1, U_sym(:,i));
    k3 = rocket.f(X_sym(:,i)-x_target+rocket.Ts/2*k2, U_sym(:,i));
    k4 = rocket.f(X_sym(:,i)-x_target+rocket.Ts*k3, U_sym(:,i));
    opti.subject_to(X_sym(:,i+1)-x_target == X_sym(:,i)-x_target + rocket.Ts/6*(k1+2*k2+2*k3+k4));

    %cost function and constraints 
    obj = obj + U_sym(:,i)'*R*U_sym(:,i)+...
         (X_sym(:,i)-x_target)'*Q*(X_sym(:,i)-x_target);
      
    opti.subject_to(M*U_sym(:,i) <= m);
    opti.subject_to(F*(X_sym(:,i)-x_target) <= f-F*x_target);

    
end

obj = obj + (X_sym(:,N)-x_target)'*Qf*(X_sym(:,N)-x_target);

opti.minimize(obj);



% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
