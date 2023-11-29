function f = objective(Z,p,U)
% computes the objective function value, based on our decision variables z
% and u

    final_leg_vel = velocity_foot(Z(:,end),p); % end effector velocity at final time
    max_body_ang = max(abs(Z(1,:))); %q1
    N = size(U,2);
    ang_vel_on_body = 0;    % torques acting on body/angular vel of body
    torques = 0;            % applied motor torques

    for t=1:N+1
        ang_vel_on_body = ang_vel_on_body + Z(5,t)^2 ; % += dq1(t) 
        
        if t < N+1 % avoid index out of range, u is dim 2xN while z is 8xN+1
        torques = torques + U(:,t)'*U(:,t);
        end
    end

    % maximize final leg vel, while minimizing ang vel on body and
    % minimizing applied torques
    f = -final_leg_vel(2) + max_body_ang + ang_vel_on_body + torques;
    %f = -final_leg_vel(2)+ang_vel_on_body;
    %f = torques;

end