function b = b_project(in1,in2,in3)
%B_PROJECT
%    B = B_PROJECT(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    01-Nov-2023 14:47:41

dq1 = in1(7,:);
dq2 = in1(8,:);
dth1 = in1(5,:);
dth2 = in1(6,:);
g = in3(19,:);
l_AB = in3(2,:);
l_Amb = in3(6,:);
l_BC = in3(3,:);
l_Bm1 = in3(7,:);
l_Cm2 = in3(8,:);
l_OA = in3(1,:);
l_Omp = in3(5,:);
m1 = in3(11,:);
m2 = in3(12,:);
mb = in3(10,:);
mp = in3(9,:);
q1 = in1(3,:);
q2 = in1(4,:);
tau1 = in2(1,:);
tau2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = sin(q1);
t3 = sin(q2);
t4 = sin(th1);
t5 = sin(th2);
t6 = q1+q2;
t7 = q1+th2;
t8 = th1+th2;
t9 = dq1.^2;
t10 = dq2.^2;
t11 = dth1.^2;
t12 = dth2.^2;
t13 = sin(t6);
t14 = t6+th2;
t15 = sin(t7);
t16 = t7+th1;
t17 = sin(t8);
t19 = t6+t8;
t25 = l_AB.*l_BC.*m2.*t2.*t9;
t26 = l_AB.*l_Bm1.*m1.*t2.*t9;
t27 = l_BC.*l_Cm2.*m2.*t3.*t10;
t28 = dq1.*dq2.*l_BC.*l_Cm2.*m2.*t3.*2.0;
t29 = dq1.*dth1.*l_AB.*l_BC.*m2.*t2.*2.0;
t30 = dq1.*dth2.*l_AB.*l_BC.*m2.*t2.*2.0;
t31 = dq1.*dth1.*l_AB.*l_Bm1.*m1.*t2.*2.0;
t32 = dq1.*dth2.*l_AB.*l_Bm1.*m1.*t2.*2.0;
t33 = dq2.*dth1.*l_BC.*l_Cm2.*m2.*t3.*2.0;
t34 = dq2.*dth2.*l_BC.*l_Cm2.*m2.*t3.*2.0;
t18 = sin(t14);
t20 = sin(t16);
t21 = sin(t19);
t22 = g.*l_AB.*m1.*t17;
t23 = g.*l_AB.*m2.*t17;
t24 = g.*l_Amb.*mb.*t17;
t41 = l_AB.*l_Cm2.*m2.*t9.*t13;
t42 = l_AB.*l_Cm2.*m2.*t10.*t13;
t43 = l_AB.*l_Cm2.*m2.*t11.*t13;
t44 = l_AB.*l_Cm2.*m2.*t12.*t13;
t45 = l_BC.*l_OA.*m2.*t11.*t15;
t46 = l_Bm1.*l_OA.*m1.*t11.*t15;
t47 = dq1.*dq2.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t48 = dq1.*dth1.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t49 = dq1.*dth2.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t50 = dq2.*dth1.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t51 = dq2.*dth2.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t52 = dth1.*dth2.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t35 = g.*l_BC.*m2.*t20;
t36 = g.*l_Bm1.*m1.*t20;
t37 = g.*l_Cm2.*m2.*t21;
t38 = -t22;
t39 = -t23;
t40 = -t24;
t55 = -t52;
t57 = l_Cm2.*l_OA.*m2.*t11.*t18;
t58 = -t43;
t59 = -t44;
t60 = -t45;
t61 = -t46;
t53 = -t35;
t54 = -t36;
t56 = -t37;
t62 = -t57;
et1 = t25+t26+t27+t28+t29+t30+t31+t32+t33+t34+t38+t39+t40+t41+t42+t47+t48+t49+t50+t51+t53+t54+t56-g.*l_OA.*m1.*t4-g.*l_OA.*m2.*t4-g.*l_OA.*mb.*t4-g.*l_Omp.*mp.*t4+l_AB.*l_OA.*m1.*t5.*t12+l_AB.*l_OA.*m2.*t5.*t12+l_BC.*l_OA.*m2.*t9.*t15+l_BC.*l_OA.*m2.*t12.*t15+l_Bm1.*l_OA.*m1.*t9.*t15+l_Bm1.*l_OA.*m1.*t12.*t15+l_Cm2.*l_OA.*m2.*t9.*t18+l_Cm2.*l_OA.*m2.*t10.*t18+l_Cm2.*l_OA.*m2.*t12.*t18+l_Amb.*l_OA.*mb.*t5.*t12+dq1.*dq2.*l_Cm2.*l_OA.*m2.*t18.*2.0+dq1.*dth1.*l_BC.*l_OA.*m2.*t15.*2.0+dq1.*dth2.*l_BC.*l_OA.*m2.*t15.*2.0+dq1.*dth1.*l_Bm1.*l_OA.*m1.*t15.*2.0+dq1.*dth2.*l_Bm1.*l_OA.*m1.*t15.*2.0+dq1.*dth1.*l_Cm2.*l_OA.*m2.*t18.*2.0+dq1.*dth2.*l_Cm2.*l_OA.*m2.*t18.*2.0+dq2.*dth1.*l_Cm2.*l_OA.*m2.*t18.*2.0+dq2.*dth2.*l_Cm2.*l_OA.*m2.*t18.*2.0+dth1.*dth2.*l_AB.*l_OA.*m1.*t5.*2.0+dth1.*dth2.*l_AB.*l_OA.*m2.*t5.*2.0;
et2 = dth1.*dth2.*l_BC.*l_OA.*m2.*t15.*2.0+dth1.*dth2.*l_Bm1.*l_OA.*m1.*t15.*2.0+dth1.*dth2.*l_Cm2.*l_OA.*m2.*t18.*2.0+dth1.*dth2.*l_Amb.*l_OA.*mb.*t5.*2.0;
b = [et1+et2;t25+t26+t27+t28+t29+t30+t31+t32+t33+t34+t38+t39+t40+t41+t42+t47+t48+t49+t50+t51+t53+t54+t56+t60+t61+t62-l_AB.*l_OA.*m1.*t5.*t11-l_AB.*l_OA.*m2.*t5.*t11-l_Amb.*l_OA.*mb.*t5.*t11;t27+t28+t33+t34+t53+t54+t55+t56+t58+t59+t60+t61+t62+tau1-l_AB.*l_BC.*m2.*t2.*t11-l_AB.*l_BC.*m2.*t2.*t12-l_AB.*l_Bm1.*m1.*t2.*t11-l_AB.*l_Bm1.*m1.*t2.*t12-dth1.*dth2.*l_AB.*l_BC.*m2.*t2.*2.0-dth1.*dth2.*l_AB.*l_Bm1.*m1.*t2.*2.0;t55+t56+t58+t59+t62+tau2-l_BC.*l_Cm2.*m2.*t3.*t9-l_BC.*l_Cm2.*m2.*t3.*t11-l_BC.*l_Cm2.*m2.*t3.*t12-dq1.*dth1.*l_BC.*l_Cm2.*m2.*t3.*2.0-dq1.*dth2.*l_BC.*l_Cm2.*m2.*t3.*2.0-dth1.*dth2.*l_BC.*l_Cm2.*m2.*t3.*2.0];
