function output_data = RunRobot()
    figure(1);  clf; % Pendulum plots

    a1 = subplot(211)
    h1 = plot(0,0,0,0);
    h1(1).XData = [];
    h1(2).XData = [];
    h1(1).YData = [];
    h1(2).YData = [];
    ylabel('q1 [rad]');
    legend('True $q_1$', 'Desired $q_1$', 'Interpreter', 'latex', 'FontSize', 14);
    title('Pendulum angle');

    a2 = subplot(212)
    h2 = plot(0,0,0,0);
    h2(1).XData = [];
    h2(2).XData = [];
    h2(1).YData = [];
    h2(2).YData = [];
    ylabel('q1_dot [rad/s]');
    legend('True $\omega_1$', 'Desired $\omega_1$', 'Interpreter', 'latex', 'FontSize', 14);
    title('Pendulum ω');


    figure(2); clf; % Angles plots
    a3 = subplot(311)
    h3(1).XData = [];
    h3(2).XData = [];
    h3(1).YData = [];
    h3(2).YData = [];
    ylabel('q2 [rad]');
    legend('True $q_2$', 'Desired $q_2$', 'Interpreter', 'latex', 'FontSize', 14);
    title('Hip angle');

    a4 = subplot(312)
    h4 = plot(0,0,0,0);
    h4(1).XData = [];
    h4(2).XData = [];
    h4(1).YData = [];
    h4(2).YData = [];
    ylabel('q3 [rad]');
    legend('True $q_3$', 'Desired $q_3$', 'Interpreter', 'latex', 'FontSize', 14);
    title('Knee angle');

    a5 = subplot(313)
    h5 = plot(0,0,0,0);
    h5(1).XData = [];
    h5(2).XData = [];
    h5(1).YData = [];
    h5(2).YData = [];
    ylabel('q4 [rad]');
    legend('True $q_4$', 'Desired $q_4$', 'Interpreter', 'latex','FontSize', 14);
    title('Shoulder angle');


    figure(3); clf; % Torque plots
    a6 = subplot(311)
    h6 = plot(0,0,0,0);
    h6(1).XData = [];
    h6(2).XData = [];
    h6(1).YData = [];
    h6(2).YData = [];
    ylabel('$\tau_2$ [Nm]', 'Interpreter', 'latex', 'FontSize', 14);
    legend('True $\tau_2$', 'Desired $\tau_2$', 'Interpreter', 'latex', 'FontSize', 14);
    title('Hip torque');

    a7 = subplot(312)
    h7 = plot(0,0,0,0);
    h7(1).XData = [];
    h7(2).XData = [];
    h7(1).YData = [];
    h7(2).YData = [];
    ylabel('$\tau_3$ [Nm]', 'Interpreter', 'latex', 'FontSize', 14);
    legend('True $\tau_3$', 'Desired $\tau_3$', 'Interpreter', 'latex', 'FontSize', 14);
    title('Knee torque');

    a8 = subplot(313)
    h8 = plot(0,0,0,0);
    h8(1).XData = [];
    h8(2).XData = [];
    h8(1).YData = [];
    h8(2).YData = [];
    ylabel('$\tau_4$ [Nm]', 'Interpreter', 'latex', 'FontSize', 14);
    legend('True $\tau_4$', 'Desired $\tau_4$', 'Interpreter', 'latex', 'FontSize', 14);
    title('Shoulder torque');

    figure(4); clf; % Foot position and foot velocity plots
    a9 = subplot(311)
    h9 = plot(0,0,0,0);
    h9(1).XData = [];
    h9(2).XData = [];
    h9(1).YData = [];
    h9(2).YData = [];
    ylabel('z [m]', 'FontSize', 14);
    legend('True z', 'Desired z', 'FontSize', 14);
    title('Foot vertical position');

    a10 = subplot(312)
    h10 = plot(0,0,0,0);
    h10(1).XData = [];
    h10(2).XData = [];
    h10(1).YData = [];
    h10(2).YData = [];
    ylabel('x [m]', 'FontSize', 14);
    legend('True x', 'Desired x', 'FontSize', 14);
    title('Foot horizontal position');

    a11 = subplot(313)
    h11 = plot(0,0,0,0);
    h11(1).XData = [];
    h11(2).XData = [];
    h11(1).YData = [];
    h11(2).YData = [];
    ylabel('v [m/s]', 'FontSize', 14);
    legend('True velocity', 'Desired velocity', 'FontSize', 14);
    title('Foot velocity');



    frdm_ip  = '192.168.1.100';     % Nucleo board ip
    frdm_port= 11223;               % Nucleo board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout

    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time

        N = length(t); % N is the length of the block of new data

        h1(1).XData(end+1:end+N) = t;
        h1(2).XData(end+1:end+N) = t;
        h2(1).XData(end+1:end+N) = t;
        h2(2).XData(end+1:end+N) = t;
        h3(1).XData(end+1:end+N) = t;
        h3(2).XData(end+1:end+N) = t;
        h4(1).XData(end+1:end+N) = t;
        h4(2).XData(end+1:end+N) = t;
        h5(1).XData(end+1:end+N) = t;
        h5(2).XData(end+1:end+N) = t;
        h6(1).XData(end+1:end+N) = t;
        h6(2).XData(end+1:end+N) = t;
        h7(1).XData(end+1:end+N) = t;
        h7(2).XData(end+1:end+N) = t;
        h8(1).XData(end+1:end+N) = t;
        h8(2).XData(end+1:end+N) = t;
        h9(1).XData(end+1:end+N) = t;
        h9(2).XData(end+1:end+N) = t;
        h10(1).XData(end+1:end+N) = t;
        h10(2).XData(end+1:end+N) = t;
        h11(1).XData(end+1:end+N) = t;
        h11(2).XData(end+1:end+N) = t;

        q1 = new_data(:,2);         % q1
        q1_des = new_data(:,3);     % q1 desired
        h1(1).YData(end+1:end+N) = q1;
        h1(2).YData(end+1:end+N) = q1_des;

        w1 = new_data(:,3);         % omega_1
        w1_des = new_data(:,4);     % omega_1 desired
        h2(1).YData(end+1:end+N) = w1;
        h2(2).YData(end+1:end+N) = w1_des;

        q2 = new_data(:,5);         % q2
        q2_des = new_data(:,6);     % q2 desired
        h3(1).YData(end+1:end+N) = q2;
        h3(2).YData(end+1:end+N) = q2_des;
        
        w2 = new_data(:,7);         % omega2
        w2_des = new_data(:,8);     % omega2 desired
        

        q3 = new_data(:,9);         % q3
        q3_des = new_data(:,10);    % q3 desired
        h4(1).YData(end+1:end+N) = q3;
        h4(2).YData(end+1:end+N) = q3_des;

        w3 = new_data(:,11);        % omega3
        w3_des = new_data(:,12);    % omega3 desired
        

        q4 = new_data(:,13);        % q4
        q4_des = new_data(:,14);    % q4 desired
        h5(1).YData(end+1:end+N) = q4;
        h5(2).YData(end+1:end+N) = q4_des;

        w4 = new_data(:,15);        % omega4
        w4_des = new_data(:,16);    % omega4 desired

        tau2 = new_data(:,17);      % tau_2
        tau2_des = new_data(:,20);  % tau_2 desired
        h6(1).YData(end+1:end+N) = tau2;
        h6(2).YData(end+1:end+N) = tau2_des;

        tau3 = new_data(:,18);      % tau_3
        tau3_des = new_data(:,21);  % tau_3 desired
        h7(1).YData(end+1:end+N) = tau3;
        h7(2).YData(end+1:end+N) = tau3_des;

        tau4 = new_data(:,19);      % tau_4
        tau4_des = new_data(:,22);  % tau_4 desired
        h8(1).YData(end+1:end+N) = tau4;
        h8(2).YData(end+1:end+N) = tau4_des;

        % How can I pass this p into the function/script?
        l_AB=.083;
        l_BC=.082;
        l_CD=.088;
        l_OA=.152;
        p = [l_AB l_BC l_CD l_OA];
        r_foot = position_foot([q1 q2 q3],p); % will be a block of size N with three rows (x,y,z) 
        v_foot = velocity_foot([q1 q2 q3 w1 w2 w3],p);

        r_foot_des = position_foot([q1_des q2_des q3_des],p);
        v_foot_des = velocity_foot([q1_des q2_des q3_des w1_des w2_des w3_des],p);
        
        h9(1).YData(end+1:end+N) = r_foot(3,:);
        h9(2).YData(end+1:end+N) = r_foot_des(3,:);

        h10(1).YData(end+1:end+N) = r_foot(1,:);
        h10(2).YData(end+1:end+N) = r_foot_des(1,:);
        
        h11(1).YData(end+1:end+N) = v_foot;
        h11(2).YData(end+1:end+N) = v_foot_des;
    end
    
    q_data = load('../data/optimal_angles.mat');
    optimal_q = q_data.optimal_angles;
    w_data = load('../data/optimal_angular_velocities.mat');
    optimal_w = w_data.optimal_angular_velocities_data;
    torque_data = load('../data/optimal_torques.mat');
    optimal_torques = torque_data.optimal_torques;

    start_period = 2;
    traj_period = 0.5;
    end_period = 2;

    q0 = [0 0 0 pi];

    K = [1 1 1];
    D = [0.1 0.1 0.1];
    duty_max = 1.0;

    input = [start_period traj_period end_period q0 K D duty_max];
    input = [input reshape(optimal_angles, 1, [])];
    input = [input reshape(optimal_w, 1, [])];
    input = [input reshape(optimal_torques, 1, [])];
    output_size = 22;

    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2],'x')
    linkaxes([a3 a4 a5],'x')
    linkaxes([a6 a7 a8],'x')
    linkaxes([a9 a10 a11],'x')
end