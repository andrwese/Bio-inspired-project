function draw_plots(Z,p,U,tspan,exp)
% Draws plots for the whole timespan, z contains values for every timestep
    nz = p(end); % generalized
    plot_w_arm = nz ==4;
    %% Compute Energy
    E = energy_foot(Z,p);
    fig1=figure(1); clf
    plot(tspan,E(1:end-1));xlabel('Time (s)'); ylabel('Energy (J)');

    %% Compute foot position over time
    rD = zeros(2,length(tspan));
    vD = zeros(2,length(tspan));
    for i = 1:length(tspan)
        if plot_w_arm
        rD(:,i) = position_foot_and_arm(Z(:,i+1),p);
        vD(:,i) = velocity_foot_and_arm(Z(:,i+1),p);
        else
        rD(:,i) = position_foot(Z(:,i+1),p);
        vD(:,i) = velocity_foot(Z(:,i+1),p);
        end
    end
    
    fig2=figure(2); clf;
    plot(tspan,rD(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,rD(2,:),'b','LineWidth',2)
    xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','y'});

    fig3=figure(3); clf;
    plot(tspan,vD(1,:),'r','LineWidth',2)
    hold on
    plot(tspan,vD(2,:),'b','LineWidth',2)
    
    xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});
    
    fig4=figure(4); clf;
    plot(tspan,Z(1:nz,1:end-1))
    if plot_w_arm 
    legend('$q_1$','$q_2$','$q_3$','$q_4$','Interpreter','latex');
    else
    legend('$q_1$','$q_2$','$q_3$','Interpreter','latex');
    end
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    
    fig5=figure(5)
    plot(tspan,Z(nz+1:2*nz,1:end-1))
    if plot_w_arm
    legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','Interpreter','latex');
    else 
    legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','Interpreter','latex');
    end
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/sec)');

    fig6=figure(6)
    if plot_w_arm
    plot(tspan,U(1,:),tspan,U(2,:),tspan,U(3,:));
    legend('$\tau_1$','$\tau_2$','$\tau_3$','Interpreter','latex');
    else
    plot(tspan,U(1,:),tspan,U(2,:));
    legend('$\tau_1$','$\tau_2$','Interpreter','latex');
    end
    xlabel('Time(s)'); ylabel('Applied Motor Torques [Nm]')

    saveas(fig1,['Sim_Results/' exp '/energy.svg'],'svg');
    saveas(fig2,['Sim_Results/' exp '/position.svg'],'svg');
    saveas(fig3,['Sim_Results/' exp '/velocities.svg'],'svg');
    saveas(fig4,['Sim_Results/' exp '/angles.svg'],'svg');
    saveas(fig5,['Sim_Results/' exp '/ang_vels.svg'],'svg');
    saveas(fig6,['Sim_Results/' exp '/torques.svg'],'svg');


end