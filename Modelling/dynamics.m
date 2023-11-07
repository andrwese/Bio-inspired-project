function dz = dynamics(z,p,u)
% Expresses the system dynamics in the form dz = f(z,u).
% INPUTS:
% z - our state vector at the current time, z(t) = [q(t) dq(t)]'
% p - parameter vector

% OUPUT:
% dz - time derivative at current timestep, dz(t) = [dq(t) ddq(t)]'

    % Get mass matrix
    A = A_project(z,p);
    
    % Compute Controls
    % use the optimal u-values given by the solver for now, later we want
    % to PD-control our states towards the optimal trajectory
    tau = u; %control_law(t,z,z_des);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_project(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:4) = z(5:8);
    dz(5:8) = qdd;
end