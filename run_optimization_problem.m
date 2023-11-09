clear; clc;

addpath(genpath('casadi'))
import casadi.*
addpath("AutoDerived/");
addpath("Modelling/");
addpath("Optimization/");
addpath("Visualization/");

%%
opti = casadi.Opti();       % our optimal solver

% define parameters
param = parameters(); % retrieve parameters
p = opti.parameter(length(param), 1);
opti.set_value(p,param);

dt = 0.01;          % length of each timestep
tf = 0.5;             % final time
N = floor(tf/dt);   % number of timestep
nz = param(end);    % number of generalized coordinates

% Initial values: everything is standing still
z0 = opti.parameter(2*nz,1);
opti.set_value(z0,[0;0;0;0;0;0]);

% define decision variables
Z = opti.variable(2*nz,N+1);    % state trajectory matrix (angles and ang. vel)
q = Z(1:nz,:);                  % generalized coordinates
dq = Z(nz+1:2*nz,:);            % generalized derivatives
U = opti.variable(2,N);         % control trajectory matrix (motor torques)

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
u_max = [1 1]';          % Nm
u_min= [0; 0];           % Nm
q_max = [1 pi/2 0]';     % joint angle torques, rad
q_min = [-1 0 -pi*2/3]'; % joint angle torques, rad
final_foot_pos = position_foot(Z(:,end),p);
final_height = -0.3; 
leg_vel = velocity_foot(Z(:,:),p);


opti.subject_to(u_min <= U <= u_max);        % torque limit constraints
opti.subject_to(q_min <= Z(1:nz,:) <= q_max);% joint limit constraints
opti.subject_to(Z(:,1)==z0);                 % enforce initial values
opti.subject_to(final_foot_pos(2) == final_height); % finish with end effector at given height
%opti.subject_to(dq(2,:) >= 0); % hip joint must have pos ang vel
%opti.subject_to(leg_vel(2,:)>=0);




% Provide initial guesses for the solver
opti.set_initial(Z, ones(2*nz,N+1));
opti.set_initial(U, ones(2,N));

% Solve the NLP using IPOPT
opti.solver('ipopt');   % set numerical backend

sol = opti.solve();     % obtain solution

% Plot optimal trajectories
tspan = linspace(0,tf,N);
draw_plots(sol.value(Z),param,sol.value(U),tspan);
animateSol(tspan,sol.value(Z),param);


