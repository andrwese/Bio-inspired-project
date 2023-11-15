clear; clc;

addpath(genpath('casadi'))
import casadi.*
addpath("AutoDerived/");
addpath("Modelling/");
addpath("Optimization/");
addpath("Visualization/");


opti = casadi.Opti();       % our optimal solver

% define parameters
param = parameters(); % retrieve parameters
p = opti.parameter(length(param), 1);
opti.set_value(p,param);

dt = 0.01;          % length of each timestep
tf = 0.5;           % final time
N = floor(tf/dt);   % number of timestep
nz = param(end);    % number of generalized coordinates
solve_w_arm = nz==4;% solve optimization problem with or without arm dynamics

% Initial values: everything is standing still
z0 = opti.parameter(2*nz,1);
z0_param = zeros(2*nz,1);
if solve_w_arm % add initial condition for arm -> straight up
    z0_param(nz)=pi;
end
opti.set_value(z0,z0_param);

% define decision variables Z and U
Z = opti.variable(2*nz,N+1);    % state trajectory matrix (angles and ang. vel)
q = Z(1:nz,:);                  % generalized coordinates
dq = Z(nz+1:2*nz,:);            % generalized derivatives
U = opti.variable(nz-1,N);      % control trajectory matrix (motor torques)

% define objective function
opti.minimize(objective(Z,p,U));

% specify system dynamics, dz/dt = f(t,z,u)
if solve_w_arm
    f = @(z,u) dynamics_w_arm(z,p,u); % function handle for system dynamics, both must be vectors!
else
    f = @(z,u) dynamics(z,p,u); % function handle for system dynamics, both must be vectors!
end

% set gap-closing constraints and constraints on derivative of u
u_tol=0.01*ones(nz-1,1); % how fast we allow torques to change, i.e. constraint on du
for k=1:N
    z_next = Z(:,k) + dt*f(Z(:,k),U(:,k)); % forward euler integration
    opti.subject_to(Z(:,k+1) == z_next); % add contraints to close the gap
    if k<N % avoid index out of range
        opti.subject_to(-u_tol<=U(:,k+1)-U(:,k)<=u_tol)
    end
end

% set constraints on q, U and Z:
u_max = ones(nz-1,1);   % Nm
u_min= zeros(nz-1,1);   % Nm
final_height = -0.2; 
if solve_w_arm
    u_min(end) = -u_max(end);
    q_max = [1 pi*2/3 0 pi*4/3]';     % joint angle torques, rad
    q_min = [-1 0 -pi*2/3 pi/3]'; % joint angle torques, rad
    final_foot_pos = position_foot_and_arm(Z(:,end),p);
    leg_vel = velocity_foot_and_arm(Z(:,:),p);
    opti.subject_to(q(nz,end)==q_min(nz)); % arm must end in its minimal value
else
    q_max = [1 pi*2/3 0]';     % joint angle torques, rad
    q_min = [-1 0 -pi*2/3]';   % joint angle torques, rad
    final_foot_pos = position_foot(Z(:,end),p);
    leg_vel = velocity_foot(Z(:,:),p);
end

opti.subject_to(u_min <= U <= u_max);        % torque limit constraints
opti.subject_to(q_min <= Z(1:nz,:) <= q_max);% joint limit constraints
opti.subject_to(Z(:,1)==z0);                 % enforce initial values
opti.subject_to(final_foot_pos(2) == final_height); % finish with end effector at given height
%opti.subject_to(U(1:2,:) == [0,0]');




% Provide initial guesses for the solver
opti.set_initial(Z, ones(2*nz,N+1));
opti.set_initial(U, ones(nz-1,N));

% Solve the NLP using IPOPT
opti.solver('ipopt');   % set numerical backend

sol = opti.solve();     % obtain solution

% Plot optimal trajectories
tspan = linspace(0,tf,N);
draw_plots(sol.value(Z),param,sol.value(U),tspan);
animateSol(tspan,sol.value(Z),param);

optimal_torques = sol.value(U);
optimal_angles = sol.value(Z(2:nz,:));
save('optimal_torques.mat', 'optimal_torques');
save('optimal_angles.mat','optimal_angles')
