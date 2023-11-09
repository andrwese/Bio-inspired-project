function test_simulation()
addpath("./Optimization/");
addpath("./Modelling/");
addpath("./AutoDerived/");
addpath("./Visualization/");
    
    %% Parameter vector
    p   = parameters();
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 10;
    N = floor(tf/dt);
    nz = p(end); % number of generalized coordinates
    tspan = linspace(0, tf, N); 
    q0 = [0.1; 0; 0]; % body is hanging freely
    dq0 = [0;0;0]; % no initial velocity
    z0 = [q0;dq0];
    z_out = zeros(2*nz,N);
    z_out(:,1) = z0;
    
    for i=1:N-1
        u = [0;0]; % replace with MPC input later
        dz = dynamics(z_out(:,i), p, u);
        % Velocity update with dynamics
        z_out(:,i+1) = z_out(:,i) + dz*dt;
        
        % Position update
        z_out(1:nz,i+1) = z_out(1:nz,i) + z_out(nz+1:2*nz,i+1)*dt;
    end
    
    draw_plots(z_out, p, zeros(2,N), tspan);

    
    %% Animate Solution

    animateSol(tspan, z_out,p);
end