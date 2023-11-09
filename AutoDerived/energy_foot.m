function E = energy_foot(in1,in2)
%ENERGY_FOOT
%    E = ENERGY_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    09-Nov-2023 09:48:57

I1 = in2(15,:);
I2 = in2(16,:);
Ib = in2(14,:);
Ip = in2(13,:);
Ir = in2(17,:);
N = in2(18,:);
dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
g = in2(19,:);
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
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = q1+q2;
t5 = N.^2;
t6 = dq1.^2;
t7 = l_Omp.^2;
t8 = l_AB.*t2;
t9 = l_Amb.*t2;
t10 = l_OA.*t2;
t11 = cos(t4);
t12 = l_AB.*t3;
t13 = l_OA.*t3;
t14 = q3+t4;
t15 = sin(t4);
t16 = cos(t14);
t17 = sin(t14);
t18 = l_BC.*t11;
t19 = l_Bm1.*t11;
t20 = l_BC.*t15;
t23 = t9+t10;
t21 = l_Cm2.*t16;
t22 = l_Cm2.*t17;
t24 = t8+t10+t19;
t25 = t8+t10+t18+t21;
E = (m1.*((dq2.*t19+dq1.*t24).^2+(dq1.*(t12+t13+l_Bm1.*t15)+dq2.*l_Bm1.*t15).^2))./2.0+(mb.*(t6.*(t13+l_Amb.*t3).^2+t6.*t23.^2))./2.0+(Ib.*t6)./2.0+(Ip.*t6)./2.0+(I1.*(dq1+dq2).^2)./2.0+(m2.*((dq2.*(t18+t21)+dq3.*t21+dq1.*t25).^2+(dq2.*(t20+t22)+dq3.*t22+dq1.*(t12+t13+t20+t22)).^2))./2.0+(mp.*(t2.^2.*t6.*t7+t3.^2.*t6.*t7))./2.0+(I2.*(dq1+dq2+dq3).^2)./2.0-g.*m1.*t24-g.*m2.*t25-g.*mb.*t23+(Ir.*dq2.^2.*t5)./2.0+(Ir.*dq3.^2.*t5)./2.0-g.*l_Omp.*mp.*t2;
