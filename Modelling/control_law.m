function u = control_law(t, z, z_des)
% Computes control output according to a simple PD controller. We want
% z(3:4) (hip/knee joint angles and ang. velocities) to follow some desired
% trajectory z_des (output from the optimization problem), and apply a PD
% controller to drive the states z towards their references z_des

% INPUTS:
% t - the current timestep 
% z - states at this timestep, z = [q dq]'
% p - parameter vector
% z_des - desired trajectory for each of the leg joints, output from optimization problem

% OUTPUT:
% u - control input to the system at current timestep, i.e. inputs to the
% motors at the hip and knee joints

kp = 5;     % spring stiffness (N/rad)
kd = .5;    % damping (N/(rad/s))

z_err = z(3:4) - z_des(3:4,t);  % error in desired angles
dz_err = z(7:8) - z_des(7:8,t); % error in desired ang. velocity

u = -kp*z_err - kd*dz_err; % Applied motor torques [tau1 tau2]'


end