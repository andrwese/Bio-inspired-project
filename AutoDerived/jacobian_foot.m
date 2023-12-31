function J = jacobian_foot(in1,in2)
%JACOBIAN_FOOT
%    J = JACOBIAN_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    09-Nov-2023 09:48:59

l_AB = in2(2,:);
l_BC = in2(3,:);
l_CD = in2(4,:);
l_OA = in2(1,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = q1+q2;
t4 = cos(t3);
t5 = q3+t3;
t6 = cos(t5);
t7 = l_BC.*t4;
t8 = l_CD.*t6;
J = reshape([t7+t8+l_AB.*t2+l_OA.*t2,0.0,t7+t8,0.0],[2,2]);
