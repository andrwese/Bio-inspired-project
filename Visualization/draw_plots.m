function draw_plots(Z,p,U,tspan)
% Draws plots for the whole timespan, z contains values for every timestep
    %% Compute Energy
    E = energy_project(Z,p);
    figure(1); clf
    plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');

    %% Compute foot position over time
    rD = zeros(2,length(tspan));
    vD = zeros(2,length(tspan));
    for i = 1:length(tspan)
        rD(:,i) = position_foot(Z(:,i),p);
        vD(:,i) = velocity_foot(Z(:,i),p);
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
    plot(tspan,Z(1:4,:)*180/pi)
    legend('$q_1$','$q_2$','$q_3$','$q_4$','Interpreter','latex');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    
    figure(5)
    plot(tspan,Z(5:8,:)*180/pi)
    legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','Interpreter','latex');
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/sec)');

    figure(6)
    plot(tspan,U(1,:),tspan,U(2,:));
    legend('$\tau_1$','$\tau_2$','Interpreter','latex');
    xlabel('Time(s)'); ylabel('Applied Motor Torques [Nm]')
end