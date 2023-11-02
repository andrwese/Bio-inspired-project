%% Define parameters and keypoint vectors
syms th1 th2 dth1 dth2 ddth1 ddth2 q1 q2 dq1 dq2 ddq1 ddq2 tau1 tau2 real
syms l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1 I2 Ir N g real
% mp = pendulum mass, mb = body, m1 = link 1/thigh , m2 = link 2/leg 

% parameters 
p = [l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1 I2 Ir N g]';


% generalized coordinates
q = [th1 th2 q1 q2]';
dq = [dth1 dth2 dq1 dq2]';
ddq = [ddth1 ddth2 ddq1 ddq2]';

% Control parameters
u = [tau1 tau2]';

% Unit vectors
ihat = [1;0;0];
jhat = [0;1;0];
khat = [0;0;1];

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to keypoints 
rA = [l_OA*sin(th1); 0; -l_OA*cos(th1)]; % attachment point of body to pendulum
rB = rA + [l_AB*sin(th1+th2); 0; -l_AB*cos(th1+th2)]; % hip joint
rC = rB + [l_BC*sin(th1+th2+q1); 0; -l_BC*cos(th1+th2+q1)]; % knee joint
rD = rC + [l_CD*sin(th1+th2+q1+q2); 0; -l_CD*cos(th1+th2+q1+q2)]; % end effector

r_mp = [l_Omp*sin(th1); 0; -l_Omp*cos(th1)]; % CM of pendulum
r_mb = rA + [l_Amb*sin(th1+th2); 0; -l_Amb*cos(th1+th2)]; % CM of body
r_m1 = rB + [l_Bm1*sin(th1+th2+q1); 0; -l_Bm1*cos(th1+th2+q1)]; % CM of thigh
r_m2 = rC + [l_Cm2*sin(th1+th2+q1+q2); 0; -l_Cm2*cos(th1+th2+q1+q2)]; % CM of thigh link


% Derive time derivatives
dr_mp = ddt(r_mp);
dr_mb = ddt(r_mb);
dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);

drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);

%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 

omega1 = dth1;
omega2 = dth1 + dth2;
omega3 = dth1 + dth2 + dq1;
omega4 = dth1 + dth2 + dq1 + dq2;

T1 = (1/2)*mp * dot(dr_mp,dr_mp) + (1/2) * Ip * omega1^2;
T2 = (1/2)*mb * dot(dr_mb,dr_mb) + (1/2) * Ib * omega2^2;
T3 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * omega3^2;
T4 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * omega4^2;
T1r = (1/2)*Ir*(N*dq1)^2; % Kinetic energy of hip motor
T2r = (1/2)*Ir*(dq1 + N*dq2)^2; % Kinetic energy of knee motor

Vgp = mp*g*dot(r_mp, khat);
Vgb = mb*g*dot(r_mb, khat);
Vg1 = m1*g*dot(r_m1, khat);
Vg2 = m2*g*dot(r_m2, khat);

T = simplify(T1 + T2 + T3 + T4 + T1r + T2r);
Vg = Vgp + Vgb + Vg1 + Vg2;

%% CHECK THESE
Q_tau1 = M2Q(tau1*jhat,omega1*jhat);
Q_tau2 = M2Q(tau2*khat,omega2*khat); 
Q_tau2R= M2Q(-tau2*khat,omega1*khat);

%Q = Q_tau1+Q_tau2 + Q_tau2R;
Q = [0 0 tau1 tau2]';

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1) rB(1) rC(1) rD(1);
             rA(3) rB(3) rC(3) rD(3)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+Vg;
L = T-Vg;
eom = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;



% Rearrange Equations of Motion
A = simplify(jacobian(eom,ddq));
b = simplify(A*ddq - eom); % CHECK: something is wrong, b shouldnt contain any ddq elements

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_Joint_Sp = A;
Grav_Joint_Sp = simplify(jacobian(Vg, q)');
Corr_Joint_Sp = simplify( eom + Q - Grav_Joint_Sp - A*ddq);

% Compute foot jacobian
J = jacobian(rD,q);

% Compute ddt( J )
dJ= reshape( ddt(J(:)) , size(J) );

% Write Energy Function and Equations of Motion
z  = [q ; dq];

rD = [rD(1);rD(3)];
drD= [drD(1);drD(3)];
J  = [J(1,1) J(1,2); J(2,1), J(2,2)];
dJ = [dJ(1,1),dJ(1,2);dJ(2,1),dJ(2,2)];

% Compute mapping from joint space to operation space
%Lambda = inv(J*inv(Mass_Joint_Sp)*J');
%mu = Lambda*J*inv(Mass_Joint_Sp)*Corr_Joint_Sp - Lambda*dJ*dq;
%rho = Lambda*J*inv(Mass_Joint_Sp)*Grav_Joint_Sp;

matlabFunction(A,'file',['A_project'],'vars',{z p});
matlabFunction(b,'file',['b_project'],'vars',{z u p});
matlabFunction(E,'file',['energy_project' ],'vars',{z p});
matlabFunction(rD,'file',['position_foot'],'vars',{z p});
matlabFunction(drD,'file',['velocity_foot'],'vars',{z p});
matlabFunction(J ,'file',['jacobian_project'],'vars',{z p});
matlabFunction(dJ ,'file',['jacobian_dot_project'],'vars',{z p});

matlabFunction(Mass_Joint_Sp, 'file', 'Mass_matrix_project', 'vars', {z,p});
matlabFunction(Grav_Joint_Sp ,'file', ['Grav_project'] ,'vars',{z p});
matlabFunction(Corr_Joint_Sp ,'file', ['Corr_project']     ,'vars',{z p});
matlabFunction(keypoints,'file',['keypoints_project'],'vars',{z p});

% matlabFunction(Lambda , 'file', ['Lambda_leg'], 'vars', {z p});
% matlabFunction(mu , 'file', ['mu_leg'], 'vars', {z p});
% matlabFunction(rho , 'file', ['rho_leg'], 'vars', {z p});
