function keypoints = keypoints_project(in1,in2)
%KEYPOINTS_PROJECT
%    KEYPOINTS = KEYPOINTS_PROJECT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    01-Nov-2023 14:47:51

l_AB = in2(2,:);
l_BC = in2(3,:);
l_CD = in2(4,:);
l_OA = in2(1,:);
q1 = in1(3,:);
q2 = in1(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = l_OA.*t2;
t6 = cos(t4);
t7 = l_OA.*t3;
t8 = q1+t4;
t9 = sin(t4);
t10 = cos(t8);
t11 = q2+t8;
t12 = sin(t8);
t13 = l_AB.*t6;
t14 = l_AB.*t9;
t15 = -t5;
t16 = l_BC.*t10;
t17 = l_BC.*t12;
t18 = -t13;
t19 = -t16;
keypoints = reshape([t7,t15,t7+t14,t15+t18,t7+t14+t17,t15+t18+t19,t7+t14+t17+l_CD.*sin(t11),t15+t18+t19-l_CD.*cos(t11)],[2,4]);
