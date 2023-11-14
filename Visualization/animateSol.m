function animateSol(tspan,x,p)
    simulate_w_arm = p(end)==4;

    figure(7); clf;
    hold on
    % Prepare plot handles
    hold on
    h_OA = plot([0],[0],'LineWidth',2);
%     h_AB = plot([0],[0],'LineWidth',2);
    h_BC = plot([0],[0],'LineWidth',2);
    h_CD = plot([0],[0],'LineWidth',2);
    if simulate_w_arm
    h_AE = plot([0],[0],'LineWidth',2);
    end


    % Body parameters
    rectWidth = 0.05;
    rect_length_ext = 0.02;
    h_body = patch('XData', [0 0 0 0], 'YData', [0 0 0 0], 'FaceColor', 'blue', 'FaceAlpha', 0.3);
   
    
    xlabel('x'); ylabel('z');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 .2 -.6 .01]);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i+1); 
        if simulate_w_arm
        keypoints = keypoints_foot_and_arm(z,p);
        else
        keypoints = keypoints_foot(z,p);
        end

        rA = keypoints(:,1); % Vector to body
        rB = keypoints(:,2); % Vector to hip joint
        rC = keypoints(:,3); % Vector to knee joint
        rD = keypoints(:,4); % Vector to end effector
        if simulate_w_arm
        rE = keypoints(:,5); % Vector to end of arm
        end


        dir_body = rB - rA;
        dir_body = dir_body / norm(dir_body);
        perp_dir_body = [dir_body(2); -dir_body(1)];
        perp_dir_body = perp_dir_body / norm(perp_dir_body);

        corner1 = rA + perp_dir_body * rectWidth / 2 - dir_body*rect_length_ext/2;
        corner2 = rA - perp_dir_body * rectWidth / 2 - dir_body*rect_length_ext/2;
        corner3 = rB - perp_dir_body * rectWidth / 2 + dir_body*rect_length_ext/2;
        corner4 = rB + perp_dir_body * rectWidth / 2 + dir_body*rect_length_ext/2;

        set(h_body, 'XData', [corner1(1), corner2(1), corner3(1), corner4(1)]);
        set(h_body, 'YData', [corner1(2), corner2(2), corner3(2), corner4(2)]);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_OA,'XData',[0 rA(1)]);
        set(h_OA,'YData',[0 rA(2)]);
        
%         set(h_AB,'XData',[rA(1) rB(1)]);
%         set(h_AB,'YData',[rA(2) rB(2)]);
        
        set(h_BC,'XData',[rB(1) rC(1)]);
        set(h_BC,'YData',[rB(2) rC(2)]);
        
        set(h_CD,'XData',[rC(1) rD(1)]);
        set(h_CD,'YData',[rC(2) rD(2)]);

        if simulate_w_arm
        set(h_AE,'XData',[rA(1) rE(1)]);
        set(h_AE,'YData',[rA(2) rE(2)]);
        end

        pause(.01)
    end
end