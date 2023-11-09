function J = jacobian_foot(in1,in2)
%JACOBIAN_FOOT
%    J = JACOBIAN_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    09-Nov-2023 09:03:38

l_CD = in2(4,:);
l_OA = in2(1,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2+q3;
t3 = cos(t2);
t4 = l_CD.*t3;
J = reshape([t4+l_OA.*cos(q1),0.0,t4,0.0],[2,2]);
