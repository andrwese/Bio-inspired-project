function rD = position_foot(in1,in2)
%POSITION_FOOT
%    rD = POSITION_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    09-Nov-2023 09:48:58

l_AB = in2(2,:);
l_BC = in2(3,:);
l_CD = in2(4,:);
l_OA = in2(1,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = q1+q2;
t5 = q3+t4;
rD = [l_AB.*t3+l_OA.*t3+l_BC.*sin(t4)+l_CD.*sin(t5);-l_AB.*t2-l_OA.*t2-l_BC.*cos(t4)-l_CD.*cos(t5)];
