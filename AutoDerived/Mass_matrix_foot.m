function Mass_Joint_Sp = Mass_matrix_foot(in1,in2)
%Mass_matrix_foot
%    Mass_Joint_Sp = Mass_matrix_foot(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    09-Nov-2023 09:03:40

I1 = in2(15,:);
I2 = in2(16,:);
Ib = in2(14,:);
Ip = in2(13,:);
Ir = in2(17,:);
N = in2(18,:);
l_AB = in2(2,:);
l_Amb = in2(6,:);
l_BC = in2(3,:);
l_Bm1 = in2(7,:);
l_Cm2 = in2(8,:);
l_OA = in2(1,:);
l_Omp = in2(5,:);
m1 = in2(11,:);
m2 = in2(12,:);
mb = in2(10,:);
mp = in2(9,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q2);
t3 = cos(q3);
t4 = q2+q3;
t5 = N.^2;
t6 = l_AB.^2;
t7 = l_BC.^2;
t8 = l_Bm1.^2;
t9 = l_Cm2.^2;
t10 = l_OA.^2;
t11 = cos(t4);
t12 = Ir.*t5;
t13 = m2.*t7;
t14 = m1.*t8;
t15 = m2.*t9;
t16 = l_AB.*l_BC.*m2.*t2;
t17 = l_AB.*l_Bm1.*m1.*t2;
t18 = l_BC.*l_Cm2.*m2.*t3;
t19 = l_BC.*l_OA.*m2.*t2;
t20 = l_Bm1.*l_OA.*m1.*t2;
t21 = t18.*2.0;
t22 = l_AB.*l_Cm2.*m2.*t11;
t23 = l_Cm2.*l_OA.*m2.*t11;
t24 = I2+t15+t18;
t25 = t22+t23+t24;
t26 = I1+I2+t13+t14+t15+t16+t17+t19+t20+t21+t22+t23;
Mass_Joint_Sp = reshape([Ib+Ip+t16+t17+t19+t20+t22+t23+t26+m1.*t6+m2.*t6+m1.*t10+m2.*t10+mb.*t10+l_Amb.^2.*mb+l_Omp.^2.*mp+l_AB.*l_OA.*m1.*2.0+l_AB.*l_OA.*m2.*2.0+l_Amb.*l_OA.*mb.*2.0,t26,t25,t26,I1+I2+t12+t13+t14+t15+t21,t24,t25,t24,I2+t12+t15],[3,3]);
