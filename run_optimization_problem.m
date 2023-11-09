clear; clc;

addpath(genpath('casadi'))
import casadi.*
addpath("AutoDerived/");
addpath("Modelling/");
addpath("Optimization/");
addpath("Simulation/");

opti = casadi.Opti();       % our optimal solver

% define parameters
p = opti.parameter(19, 1);
opti.set_value(p,parameters());

dt = 0.01;          % length of each timestep
tf = 10;            % final time
N = floor(tf/dt);   % number of timesteps

% Initial values: everything is standing still
z0 = opti.parameter(8,1);
opti.set_value(z0,zeros(8,1));

% define decision variables
Z = opti.variable(8,N+1);   % state trajectory matrix (angles and ang. vel)
q = Z(1:4,:);               % generalized coordinates
dq = Z(5:8,:);              % generalized derivatives
U = opti.variable(2,N);     % control trajectory matrix (motor torques)

% define objective function
opti.minimize(objective(Z,p,U));

% specify system dynamics, dz/dt = f(t,z,u)
f = @(z,u) dynamics(z,p,u); % function handle for system dynamics, both must be vectors!

% set gap-closing constraints
for k=1:N
    z_next = Z(:,k) + dt*f(Z(:,k),U(:,k)); % forward euler integration
    opti.subject_to(Z(:,k+1) == z_next); % add contraints to close the gap
end

% set constraints on q, U and Z:
u_max = [10 10]'; % ------- CHECK ------------ enhet??
q_max = [1 1 pi/2 pi/2]'; % joint angle torques, rad
final_foot_pos = position_foot(Z(:,end),p);
final_height = -0.3; % ---- CHECK ------ find reasonable values

% opti.subject_to(-u_max <= U <= u_max);        % torque limit constraints
% opti.subject_to(-q_max <= Z(1:4,:) <= q_max); % joint limit constraints
% opti.subject_to(Z(:,1)==z0);             % enforce initial values
% opti.subject_to(final_foot_pos(2) == final_height); % finish with end effector at given height

% Provide initial guesses for the solver
%opti.set_initial(); % TODO

% Solve the NLP using IPOPT
opti.solver('ipopt');   % set numerical backend
sol = opti.solve();     % obtain solution

% Plot optimal trajectories
tspan = linspace(0,tf,N);
draw_plots(sol.value(Z),p,sol.value(U),tspan);
animateSol(tspan,sol.value(Z),p);


