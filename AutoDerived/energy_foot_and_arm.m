function E = energy_foot_and_arm(in1,in2)
%ENERGY_FOOT_AND_ARM
%    E = ENERGY_FOOT_AND_ARM(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    14-Nov-2023 13:38:59

I1 = in2(18,:);
I2 = in2(19,:);
I3 = in2(20,:);
Ib = in2(17,:);
Ip = in2(16,:);
Ir = in2(21,:);
N = in2(22,:);
dq1 = in1(5,:);
dq2 = in1(6,:);
dq3 = in1(7,:);
dq4 = in1(8,:);
g = in2(23,:);
l_AB = in2(2,:);
l_Am3 = in2(10,:);
l_Amb = in2(7,:);
l_BC = in2(3,:);
l_Bm1 = in2(8,:);
l_Cm2 = in2(9,:);
l_OA = in2(1,:);
l_Omp = in2(6,:);
m1 = in2(13,:);
m2 = in2(14,:);
m3 = in2(15,:);
mb = in2(12,:);
mp = in2(11,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = q1+q2;
t5 = q1+q4;
t6 = N.^2;
t7 = dq1.^2;
t8 = l_Omp.^2;
t9 = l_AB.*t2;
t10 = l_Amb.*t2;
t11 = l_OA.*t2;
t12 = cos(t4);
t13 = cos(t5);
t14 = l_AB.*t3;
t15 = l_OA.*t3;
t16 = q3+t4;
t17 = sin(t4);
t18 = sin(t5);
t19 = cos(t16);
t20 = sin(t16);
t21 = l_Am3.*t13;
t22 = l_BC.*t12;
t23 = l_Bm1.*t12;
t24 = l_BC.*t17;
t27 = t10+t11;
t25 = l_Cm2.*t19;
t26 = l_Cm2.*t20;
t28 = t11+t21;
t29 = t9+t11+t23;
t30 = t9+t11+t22+t25;
et1 = (m1.*((dq2.*t23+dq1.*t29).^2+(dq1.*(t14+t15+l_Bm1.*t17)+dq2.*l_Bm1.*t17).^2))./2.0+(mb.*(t7.*(t15+l_Amb.*t3).^2+t7.*t27.^2))./2.0+(Ib.*t7)./2.0+(Ip.*t7)./2.0+(I1.*(dq1+dq2).^2)./2.0+(I3.*(dq1+dq4).^2)./2.0+(m2.*((dq2.*(t22+t25)+dq3.*t25+dq1.*t30).^2+(dq2.*(t24+t26)+dq3.*t26+dq1.*(t14+t15+t24+t26)).^2))./2.0+(mp.*(t2.^2.*t7.*t8+t3.^2.*t7.*t8))./2.0+(I2.*(dq1+dq2+dq3).^2)./2.0+(m3.*((dq4.*t21+dq1.*t28).^2+(dq1.*(t15+l_Am3.*t18)+dq4.*l_Am3.*t18).^2))./2.0-g.*m1.*t29-g.*m3.*t28-g.*m2.*t30;
et2 = -g.*mb.*t27+(Ir.*dq2.^2.*t6)./2.0+(Ir.*dq3.^2.*t6)./2.0+(Ir.*dq4.^2.*t6)./2.0-g.*l_Omp.*mp.*t2;
E = et1+et2;
