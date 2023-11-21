function dz = dynamics_w_arm(z,p,u)
% Expresses the system dynamics in the form dz = f(z,u).
% INPUTS:
% t - current timestep
% z - our state vector at the current time, z(t) = [q(t) dq(t)]'
% p - parameter vector
% u - control points for the bezier trajectory (3xNu matrix bc. we have 3
% control inputs)

% OUPUT:
% dz - time derivative at current timestep, dz(t) = [dq(t) ddq(t)]'

    nz = 4; % p(end); % number of generalized coordinates
    % Get mass matrix
    A = A_foot_and_arm(z,p);
    
    % Compute Controls
    tau = u;
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_foot_and_arm(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:nz) = z(nz+1:2*nz);
    dz(nz+1:2*nz) = qdd;
end