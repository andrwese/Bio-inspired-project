function animateSol(tspan, x,p)
    figure(7); clf;
    hold on
    % Prepare plot handles
    hold on
    h_OA = plot([0],[0],'LineWidth',2);
    h_AB = plot([0],[0],'LineWidth',2);
    h_BC = plot([0],[0],'LineWidth',2);
    h_CD = plot([0],[0],'LineWidth',2);
   
    
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
        z = x(:,i); 
        keypoints = keypoints_foot(z,p);

        rA = keypoints(:,1); % Vector to body
        rB = keypoints(:,2); % Vector to hip joint
        rC = keypoints(:,3); % Vector to knee joint
        rD = position_foot(z,p);%keypoints(:,4); % Vector to end effector

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