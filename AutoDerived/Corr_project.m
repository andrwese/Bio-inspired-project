function Corr_Joint_Sp = Corr_project(in1,in2)
%Corr_project
%    Corr_Joint_Sp = Corr_project(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    07-Nov-2023 12:58:18

dq1 = in1(5,:);
dq2 = in1(6,:);
dq3 = in1(7,:);
dq4 = in1(8,:);
l_AB = in2(2,:);
l_Amb = in2(6,:);
l_BC = in2(3,:);
l_Bm1 = in2(7,:);
l_Cm2 = in2(8,:);
l_OA = in2(1,:);
m1 = in2(11,:);
m2 = in2(12,:);
mb = in2(10,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
t2 = sin(q2);
t3 = sin(q3);
t4 = sin(q4);
t5 = q2+q3;
t6 = q3+q4;
t7 = dq1.^2;
t8 = dq2.^2;
t9 = dq3.^2;
t10 = dq4.^2;
t11 = q4+t5;
t12 = sin(t5);
t13 = sin(t6);
t15 = l_AB.*l_BC.*m2.*t3.*t9;
t16 = l_AB.*l_Bm1.*m1.*t3.*t9;
t17 = l_BC.*l_Cm2.*m2.*t4.*t10;
t18 = dq1.*dq3.*l_AB.*l_BC.*m2.*t3.*2.0;
t19 = dq2.*dq3.*l_AB.*l_BC.*m2.*t3.*2.0;
t20 = dq1.*dq3.*l_AB.*l_Bm1.*m1.*t3.*2.0;
t21 = dq2.*dq3.*l_AB.*l_Bm1.*m1.*t3.*2.0;
t22 = dq1.*dq4.*l_BC.*l_Cm2.*m2.*t4.*2.0;
t23 = dq2.*dq4.*l_BC.*l_Cm2.*m2.*t4.*2.0;
t24 = dq3.*dq4.*l_BC.*l_Cm2.*m2.*t4.*2.0;
t14 = sin(t11);
t25 = -t18;
t26 = -t19;
t27 = -t20;
t28 = -t21;
t29 = -t22;
t30 = -t23;
t31 = -t24;
t32 = l_AB.*l_Cm2.*m2.*t9.*t13;
t33 = l_AB.*l_Cm2.*m2.*t10.*t13;
t34 = l_BC.*l_OA.*m2.*t7.*t12;
t35 = l_Bm1.*l_OA.*m1.*t7.*t12;
t36 = dq1.*dq3.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t37 = dq1.*dq4.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t38 = dq2.*dq3.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t39 = dq2.*dq4.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t40 = dq3.*dq4.*l_AB.*l_Cm2.*m2.*t13.*2.0;
t41 = -t15;
t42 = -t16;
t43 = -t17;
t44 = -t36;
t45 = -t37;
t46 = -t38;
t47 = -t39;
t48 = -t40;
t49 = l_Cm2.*l_OA.*m2.*t7.*t14;
t50 = -t32;
t51 = -t33;
et1 = t25+t26+t27+t28+t29+t30+t31+t41+t42+t43+t44+t45+t46+t47+t48+t50+t51-l_AB.*l_OA.*m1.*t2.*t8-l_AB.*l_OA.*m2.*t2.*t8-l_BC.*l_OA.*m2.*t8.*t12-l_BC.*l_OA.*m2.*t9.*t12-l_Bm1.*l_OA.*m1.*t8.*t12-l_Bm1.*l_OA.*m1.*t9.*t12-l_Cm2.*l_OA.*m2.*t8.*t14-l_Cm2.*l_OA.*m2.*t9.*t14-l_Cm2.*l_OA.*m2.*t10.*t14-l_Amb.*l_OA.*mb.*t2.*t8-dq1.*dq2.*l_AB.*l_OA.*m1.*t2.*2.0-dq1.*dq2.*l_AB.*l_OA.*m2.*t2.*2.0-dq1.*dq2.*l_BC.*l_OA.*m2.*t12.*2.0-dq1.*dq3.*l_BC.*l_OA.*m2.*t12.*2.0-dq2.*dq3.*l_BC.*l_OA.*m2.*t12.*2.0-dq1.*dq2.*l_Bm1.*l_OA.*m1.*t12.*2.0-dq1.*dq3.*l_Bm1.*l_OA.*m1.*t12.*2.0-dq2.*dq3.*l_Bm1.*l_OA.*m1.*t12.*2.0-dq1.*dq2.*l_Cm2.*l_OA.*m2.*t14.*2.0-dq1.*dq3.*l_Cm2.*l_OA.*m2.*t14.*2.0-dq1.*dq4.*l_Cm2.*l_OA.*m2.*t14.*2.0;
et2 = dq2.*dq3.*l_Cm2.*l_OA.*m2.*t14.*-2.0-dq2.*dq4.*l_Cm2.*l_OA.*m2.*t14.*2.0-dq3.*dq4.*l_Cm2.*l_OA.*m2.*t14.*2.0-dq1.*dq2.*l_Amb.*l_OA.*mb.*t2.*2.0;
Corr_Joint_Sp = [et1+et2;t25+t26+t27+t28+t29+t30+t31+t34+t35+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+l_AB.*l_OA.*m1.*t2.*t7+l_AB.*l_OA.*m2.*t2.*t7+l_Amb.*l_OA.*mb.*t2.*t7;t29+t30+t31+t34+t35+t43+t49+l_AB.*l_BC.*m2.*t3.*t7+l_AB.*l_BC.*m2.*t3.*t8+l_AB.*l_Bm1.*m1.*t3.*t7+l_AB.*l_Bm1.*m1.*t3.*t8+l_AB.*l_Cm2.*m2.*t7.*t13+l_AB.*l_Cm2.*m2.*t8.*t13+dq1.*dq2.*l_AB.*l_BC.*m2.*t3.*2.0+dq1.*dq2.*l_AB.*l_Bm1.*m1.*t3.*2.0+dq1.*dq2.*l_AB.*l_Cm2.*m2.*t13.*2.0;l_Cm2.*m2.*(l_AB.*t7.*t13+l_AB.*t8.*t13+l_BC.*t4.*t7+l_BC.*t4.*t8+l_BC.*t4.*t9+l_OA.*t7.*t14+dq1.*dq2.*l_AB.*t13.*2.0+dq1.*dq2.*l_BC.*t4.*2.0+dq1.*dq3.*l_BC.*t4.*2.0+dq2.*dq3.*l_BC.*t4.*2.0)];
