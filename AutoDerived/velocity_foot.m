function drD = velocity_foot(in1,in2)
%VELOCITY_FOOT
%    drD = VELOCITY_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    09-Nov-2023 09:48:58

dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
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
t5 = cos(t4);
t6 = q3+t4;
t7 = sin(t4);
t8 = cos(t6);
t9 = sin(t6);
t10 = l_BC.*t5;
t11 = l_BC.*t7;
t12 = l_CD.*t8;
t13 = l_CD.*t9;
drD = [dq2.*(t10+t12)+dq3.*t12+dq1.*(t10+t12+l_AB.*t2+l_OA.*t2);dq2.*(t11+t13)+dq3.*t13+dq1.*(t11+t13+l_AB.*t3+l_OA.*t3)];
