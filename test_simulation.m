function test_simulation()
addpath("./Optimization/");
addpath("./Modelling/");
addpath("./AutoDerived/");
    %% Define fixed paramters
    m1 =.0393 + .2;         m2 =.0368; 
    mp = .00783;            mb = 1.55;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    Ip = 9.25 * 10^-6;      Ib = 22.176 * 10^-6;
    l_OA=.11;               l_AB=.042; 
    l_BC=.096;              l_CD=.087;
    l_Omp=l_OA/2;           l_Amb=0.01; 
    l_Bm1=0.0622;           l_Cm2=0.0610;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;    
    
    %% Parameter vector
    p   = [l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1 I2 Ir N g]';
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 10;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step); 
    q0 = [0.1; 0; 0; 0]; % body is hanging freely
    dq0 = [0;0;0;0]; % no initial velocity
    z0 = [q0;dq0];
    z_out = zeros(8,num_step);
    z_out(:,1) = z0;
    
    for i=1:num_step-1
        u = [0;0];
        dz = dynamics(z_out(:,i), p, u);
        % Velocity update with dynamics
        z_out(:,i+1) = z_out(:,i) + dz*dt;
        
        % Position update
        z_out(1:4,i+1) = z_out(1:4,i) + z_out(5:8,i+1)*dt;
    end
    
    %% Compute Energy
    E = energy_project(z_out,p);
    figure(1); clf
    plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');
    
    %% Compute foot position over time
    rD = zeros(2,length(tspan));
    vD = zeros(2,length(tspan));
    for i = 1:length(tspan)
        rD(:,i) = position_foot(z_out(:,i),p);
        vD(:,i) = velocity_foot(z_out(:,i),p);
    end
    
    figure(2); clf;
    plot(tspan,rD(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,rD(2,:),'b','LineWidth',2)
    xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','y'});

    figure(3); clf;
    plot(tspan,vD(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,vD(2,:),'b','LineWidth',2)
    
    xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});
    
    figure(4)
    plot(tspan,z_out(1:4,:)*180/pi)
    legend('$\theta_1$','$\theta_2$','$q_1$','$q_2$','Interpreter','latex');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    
    figure(5)
    plot(tspan,z_out(5:8,:)*180/pi)
    legend('$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{q}_1$','$\dot{q}_2$','Interpreter','latex');
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/sec)');
    
    %% Animate Solution
    figure(6); clf;
    hold on
    
    animateSol(tspan, z_out,p);
end