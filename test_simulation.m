function test_simulation()
addpath("./Optimization/");
addpath("./Modelling/");
addpath("./AutoDerived/");
addpath("./Visualization/");
addpath("./data");
    
    %% Parameter vector
    dt = 0.01;
    tf = 0.5;
    N = floor(tf/dt);
    p   = parameters();

    %% Retrieve torques
    bezier_pts = load("data/bezier_pts.mat");
    bezier_torques = compute_Bezier_Trajectory(dt,tf,bezier_pts.bezier_pts);

    poly_coeffs = load("data/poly_coeffs.mat");
    polynomial_torques = compute_polynomial_trajectory(dt,tf,poly_coeffs.poly_coeffs);
    %% Perform Dynamic simulation
    nz = p(end); % number of generalized coordinates
    tspan = linspace(0, tf, N); 
    q0 = [0; zeros(nz-1,1)]; % body is hanging freely
    if nz==4 % arm is not aligned with body
        q0(end)=pi;
    end
 
    dq0 = zeros(nz,1); % no initial velocity
    z0 = [q0;dq0];
    z_out = zeros(2*nz,N+1);
    z_out(:,1) = z0;
    
    for i=1:N
        %u = bezier_torques(:,i);
        u = polynomial_torques(:,i);

        if nz==4
        dz = dynamics_w_arm(z_out(:,i), p, u);
        else
        dz = dynamics(z_out(:,i), p, u);
        end
        % Velocity update with dynamics
        z_out(:,i+1) = z_out(:,i) + dz*dt;
        
        % Position update
        z_out(1:nz,i+1) = z_out(1:nz,i) + z_out(nz+1:2*nz,i+1)*dt;
    end
    
    draw_plots(z_out, p, zeros(nz-1,N),tspan);

    
    %% Animate Solution

    animateSol(tspan, z_out,p);
end