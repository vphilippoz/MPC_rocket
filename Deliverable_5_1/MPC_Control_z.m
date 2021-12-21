classdef MPC_Control_z < MPC_Control
    properties
        A_bar, B_bar, C_bar % Augmented system for disturbance rejection
        L                   % Estimator gain for disturbance rejection
    end
    
    methods
        function mpc = MPC_Control_z(sys, Ts, H)
            mpc = mpc@MPC_Control(sys, Ts, H);
            
            [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
        end
        
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   d_est        - disturbance estimate
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps
            
            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.3)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar(1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % Cost matrices 
            Q = 50*eye(nx);
            R = eye(nu);
            [~, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R);
            
        
            % Constraints
            %   State constraints
            % no state constraints
            
            %  Input contstraints
            M = [1; -1];
            m = [80 - 50.667; -(50 - 50.667)]; %substract steady state input
            
            
            % Constraints and objective
            con = ((X(:,2)-x_ref) == mpc.A*(X(:,1)-x_ref) + mpc.B*(U(:,1)-u_ref));
            con = con + (M*(U(:,1)-u_ref) <= m - M*u_ref);
            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);
            for i = 2:N-1
                con = con + ((X(:,i+1)-x_ref) == mpc.A*(X(:,i)-x_ref) + mpc.B*(U(:,i)-u_ref));
                con = con + (M*(U(:,i)-u_ref) <= m - M*u_ref);
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref);
                obj = obj + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);


            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref, d_est}, U(:,1));
        end
        
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);
            
            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.3)
            ref = sdpvar;
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            
            nx = size(mpc.A);
            nu = size(mpc.B,2);

            % Input cost
            Rs = eye(nu);

            % Input constraints
            M = [1; -1];
            m = [80-56.667; -50+56.667]; %substract us
            
            % Setup objective and constraints
            mat = [eye(nx)-mpc.A, -mpc.B; mpc.C, 0];
            obj = us'*Rs*us;
            con = (mat*[xs; us] == [mpc.B*d_est; ref]) + (M*us <= m);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
        end
        
        
        % Compute augmented system and estimator gain for input disturbance rejection
        function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
            
            %%% Design the matrices A_bar, B_bar, L, and C_bar
            %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
            %%% converges to the correct state and constant input disturbance
            %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            nx = size(mpc.A,1);
            nu = size(mpc.B,2);
            nd = 1;
            
            A_bar = [mpc.A, mpc.B;
                     zeros(nd,nx), eye(nd,nu)];
            B_bar = [mpc.B; zeros(1,nu)];
            C_bar = [mpc.C, zeros(1,nu)];

            poles = [0.7, 0.8, 0.9]';
            L = -place(A_bar',C_bar', poles)';
          
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        
    end
end
