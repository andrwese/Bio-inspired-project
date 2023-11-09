function Corr_Joint_Sp = Corr_foot(in1,in2)
%Corr_foot
%    Corr_Joint_Sp = Corr_foot(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    09-Nov-2023 09:03:42

dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
l_AB = in2(2,:);
l_BC = in2(3,:);
l_Bm1 = in2(7,:);
l_Cm2 = in2(8,:);
l_OA = in2(1,:);
m1 = in2(11,:);
m2 = in2(12,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = sin(q2);
t3 = sin(q3);
t4 = q2+q3;
t5 = dq1.^2;
t6 = dq2.^2;
t7 = dq3.^2;
t8 = sin(t4);
t9 = l_BC.*l_Cm2.*m2.*t3.*t7;
t10 = dq1.*dq3.*l_BC.*l_Cm2.*m2.*t3.*2.0;
t11 = dq2.*dq3.*l_BC.*l_Cm2.*m2.*t3.*2.0;
t12 = -t10;
t13 = -t11;
t14 = -t9;
mt1 = [t12+t13+t14-l_AB.*l_BC.*m2.*t2.*t6-l_AB.*l_Bm1.*m1.*t2.*t6-l_AB.*l_Cm2.*m2.*t6.*t8-l_AB.*l_Cm2.*m2.*t7.*t8-l_BC.*l_OA.*m2.*t2.*t6-l_Bm1.*l_OA.*m1.*t2.*t6-l_Cm2.*l_OA.*m2.*t6.*t8-l_Cm2.*l_OA.*m2.*t7.*t8-dq1.*dq2.*l_AB.*l_BC.*m2.*t2.*2.0-dq1.*dq2.*l_AB.*l_Bm1.*m1.*t2.*2.0-dq1.*dq2.*l_AB.*l_Cm2.*m2.*t8.*2.0-dq1.*dq3.*l_AB.*l_Cm2.*m2.*t8.*2.0-dq2.*dq3.*l_AB.*l_Cm2.*m2.*t8.*2.0-dq1.*dq2.*l_BC.*l_OA.*m2.*t2.*2.0-dq1.*dq2.*l_Bm1.*l_OA.*m1.*t2.*2.0-dq1.*dq2.*l_Cm2.*l_OA.*m2.*t8.*2.0-dq1.*dq3.*l_Cm2.*l_OA.*m2.*t8.*2.0-dq2.*dq3.*l_Cm2.*l_OA.*m2.*t8.*2.0;t12+t13+t14+l_AB.*l_BC.*m2.*t2.*t5+l_AB.*l_Bm1.*m1.*t2.*t5+l_AB.*l_Cm2.*m2.*t5.*t8+l_BC.*l_OA.*m2.*t2.*t5+l_Bm1.*l_OA.*m1.*t2.*t5+l_Cm2.*l_OA.*m2.*t5.*t8];
mt2 = [l_Cm2.*m2.*(l_AB.*t5.*t8+l_BC.*t3.*t5+l_BC.*t3.*t6+l_OA.*t5.*t8+dq1.*dq2.*l_BC.*t3.*2.0)];
Corr_Joint_Sp = [mt1;mt2];
