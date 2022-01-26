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

%creating a symbolic function for the rocket dynamics
f_symbolic = @(X_, U_) rocket.f(X_,U_);


%cost of the states (cost =x'* Q* x)
Q = 100*eye(nx);

%terminal cost based on the lineraization of the system
[xs, us] = rocket.trim();
sys = rocket.linearize(xs,us);
sys_d = c2d(sys, rocket.Ts);


Rx = 5;       %cost of the inputs
Ry = 5;
Rz = 10;
Rroll = 15;
R = diag([Rx Ry Rz Rroll]);

%computing the LQR cost as a terminal cost
[~,Qf,~] = dlqr(sys_d.A, sys_d.B, Q, R);

%setup the constraints 
%state constraints F*x <= f
F = [0 0 0 1 0 0 0 0 0 0 0 0;...   %constraints on alpha
     0 0 0 -1 0 0 0 0 0 0 0 0;...
     0 0 0 0 1 0 0 0 0 0 0 0;...   %constraints on beta
     0 0 0 0 -1 0 0 0 0 0 0 0];

f = [deg2rad(85); deg2rad(85); deg2rad(85); deg2rad(85)];

%input constraints M*u <= m
M =[1 0 0 0;...    %d1 constraints
    -1 0 0 0;...
    0 1 0 0;...   %d2 constraints
    0 -1 0 0;...
    0 0 1 0;...    %P_avg constraints
    0 0 -1 0;...
    0 0 0 1;...   %Pdiff constraints
    0 0 0 -1];

m = [deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15);...
     80; -50; 20; 20];

%now we can implement the cost, we assume that the at the steady state, the
%rocket velocities are 0 and (since x_dot = 0 at steady state => all
%rotational and displacement speeds are 0) We also assume that the rocket
%is aligned with the z axis (alpha = beta = 0)

%target state
x_target = [0 0 0 0 0 ref_sym(4) 0 0 0 ref_sym(1) ref_sym(2) ref_sym(3)]';


opti.minimize(250*X_sym(1,2:N-1)*X_sym(1,2:N-1)'+... %wx
              250*X_sym(2,2:N-1)*X_sym(2,2:N-1)'+... %wy
              100*X_sym(3,2:N-1)*X_sym(3,2:N-1)'+... %wz
              50*X_sym(4,2:N-1)*X_sym(4,2:N-1)'+... %alpha
              50*X_sym(5,2:N-1)*X_sym(5,2:N-1)'+... %beta
              200*(X_sym(6,2:N-1)-ref_sym(4))*(X_sym(6,2:N-1)-ref_sym(4))'+... %gamma
              20*X_sym(7,2:N-1)*X_sym(7,2:N-1)'+... %vx
              20*X_sym(8,2:N-1)*X_sym(8,2:N-1)'+... %vy
              20*X_sym(9,2:N-1)*X_sym(9,2:N-1)'+... %vz
              350*(X_sym(10,2:N-1)-ref_sym(1))*(X_sym(10,2:N-1)-ref_sym(1))'+... %x
              350*(X_sym(11,2:N-1)-ref_sym(2))*(X_sym(11,2:N-1)-ref_sym(2))'+... %y
              350*(X_sym(12,2:N-1)-ref_sym(3))*(X_sym(12,2:N-1)-ref_sym(3))'+... %z
              1.5*U_sym(1,:)*U_sym(1,:)'+... 
              1.5*U_sym(2,:)*U_sym(2,:)'+...
              1.5*(U_sym(3,:)-us(3))*(U_sym(3,:)-us(3))'+... 
              1.5*U_sym(4,:)*U_sym(4,:)'+...
              (X_sym(:,N)-x_target)'*Qf*(X_sym(:,N)-x_target));
              

                 

opti.subject_to(X_sym(:,1) == x0_sym);
%compute the next state using a RK4 integration function
next_state = RK4(X_sym(:,1), U_sym(:,1), rocket.Ts, f_symbolic);

opti.subject_to(M*U_sym(:,1) <= m);
opti.subject_to(X_sym(:,2) == next_state);


for i = 2:N-1
    
    next_state = RK4(X_sym(:,i), U_sym(:,i), rocket.Ts, f_symbolic);
    opti.subject_to((X_sym(:,i+1)) ==  next_state);
    
    opti.subject_to(M*U_sym(:,i) <= m);
    opti.subject_to(F*(X_sym(:,i)-x_target) <= f-F*x_target);
    
end
    opti.subject_to(F*(X_sym(:,N)-x_target) <= f-F*x_target);

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
