function simulate()
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
    p_traj = 0; % change later
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
        dz = dynamics(tspan(i), z_out(:,i), p, p_traj);
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

function tau = control_law(t, z, p, p_traj)
    tau = [0;0];
end


function dz = dynamics(t,z,p,p_traj)
    % Get mass matrix
    A = A_project(z,p);
    
    % Compute Controls
    tau = control_law(t,z,p,p_traj);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_project(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:4) = z(5:8);
    dz(5:8) = qdd;
end



function qdot = joint_limit_constraint(z,p)
    %% Fixed parameters for rotational spring damper at joint limit contact
    qdot = z(3:4);
    q1_c = -50*pi/180; % constraint angle (rad)
    q1 = z(1);

    C = q1 - q1_c;
    if C > 0 % ignore constraint torque if constraint is not violated
        return
    end
  
    % computing a constraint torque (spring damper)
    Kappa_c = 0.1;
    Dampa_c = 0.002;

    dq1 = z(3);
    dC = dq1;

    Tau_c = -Kappa_c*C - Dampa_c*dC;

    % update equation using operational space matrix
    % Note: J'F = torques, so we dont need J here since we already have
    % Tau_c as a torque
    M = Mass_matrix_project(z,p);
    qdot = qdot + inv(M)*[Tau_c; 0];
end

function animateSol(tspan, x,p)
    % Prepare plot handles
    hold on
    h_OA = plot([0],[0],'LineWidth',2);
    h_AB = plot([0],[0],'LineWidth',2);
    h_BC = plot([0],[0],'LineWidth',2);
    h_CD = plot([0],[0],'LineWidth',2);
   
    
    xlabel('x'); ylabel('z');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 .2 -.3 .1]);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_project(z,p);

        rA = keypoints(:,1); % Vector to body
        rB = keypoints(:,2); % Vector to hip joint
        rC = keypoints(:,3); % Vector to knee joint
        rD = keypoints(:,4); % Vector to end effector

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_OA,'XData',[0 rA(1)]);
        set(h_OA,'YData',[0 rA(2)]);
        
        set(h_AB,'XData',[rA(1) rB(1)]);
        set(h_AB,'YData',[rA(2) rB(2)]);
        
        set(h_BC,'XData',[rB(1) rC(1)]);
        set(h_BC,'YData',[rB(2) rC(2)]);
        
        set(h_CD,'XData',[rC(1) rD(1)]);
        set(h_CD,'YData',[rC(2) rD(2)]);

        pause(.01)
    end
end