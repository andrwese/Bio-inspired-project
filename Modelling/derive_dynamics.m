% Derives equations of motion for the system, and collects relevant
% variables in exported functions

%% Define parameters and keypoint vectors
syms q1 q2 dq1 dq2 ddq1 ddq2 q3 q4 dq3 dq4 ddq3 ddq4 tau1 tau2 real
syms l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1 I2 Ir N g nz real
% mp = pendulum mass, mb = body, m1 = link 1/thigh , m2 = link 2/leg 

% parameters 
p = [l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1 I2 Ir N g nz]';

% generalized coordinates
q = [q1 q2 q3]';            % pendulum, hip, knee, shoulder=q4
dq = [dq1 dq2 dq3 ]';       % dq4
ddq = [ddq1 ddq2 ddq3 ]';   % ddq4

% Control parameters
u = [tau1 tau2]'; % hip, knee, shoulder

% Unit vectors
ihat = [1;0;0];
jhat = [0;1;0];
khat = [0;0;1];

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to keypoints 
rA = [l_OA*sin(q1); 0; -l_OA*cos(q1)]; % pendulum to body
rB = rA + [l_AB*sin(q1); 0; -l_AB*cos(q1)]; % body to hip joint (just an extension of rA)
rC = rB + [l_BC*sin(q1+q2); 0; -l_BC*cos(q1+q2)]; % knee joint
rD = rC + [l_CD*sin(q1+q2+q3); 0; -l_CD*cos(q1+q2+q3)]; % end effector
% rE = rB + [l_BE*sin(q1+q4);0;-l_BE*cos(q1+q4)]; % arm

r_mp = [l_Omp*sin(q1); 0; -l_Omp*cos(q1)]; % CM of pendulum
r_mb = rA + [l_Amb*sin(q1); 0; -l_Amb*cos(q1)]; % CM of body
r_m1 = rB + [l_Bm1*sin(q1+q2); 0; -l_Bm1*cos(q1+q2)]; % CM of thigh
r_m2 = rC + [l_Cm2*sin(q1+q2+q3); 0; -l_Cm2*cos(q1+q2+q3)]; % CM of thigh link
%r_m3 = rB + [l_Bm3*sin(q1+q4); 0; -l_Bm3*cos(q1+q4)]; % CM of arm

% Derive time derivatives
dr_mp = ddt(r_mp);
dr_mb = ddt(r_mb);
dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);
%dr_m3 = ddt(r_m3);

drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
% drE = ddt(rE);

%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 

omega1 = dq1;               % ang vel of pendulum
omega2 = dq1;               % ang vel of body
omega3 = dq1 + dq2;         % ang vel of thigh
omega4 = dq1 + dq2 + dq3;   % ang vel of leg

T1 = (1/2)*mp * dot(dr_mp,dr_mp) + (1/2) * Ip * omega1^2;
T2 = (1/2)*mb * dot(dr_mb,dr_mb) + (1/2) * Ib * omega2^2;
T3 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * omega3^2;
T4 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * omega4^2;
T1r = (1/2)*Ir*(N*dq2)^2; % Kinetic energy of hip motor
T2r = (1/2)*Ir*(N*dq3)^2; % Kinetic energy of knee motor

Vgp = mp*g*dot(r_mp, khat);
Vgb = mb*g*dot(r_mb, khat);
Vg1 = m1*g*dot(r_m1, khat);
Vg2 = m2*g*dot(r_m2, khat);

T = simplify(T1 + T2 + T3 + T4 + T1r + T2r);
Vg = Vgp + Vgb + Vg1 + Vg2;

%% CHECK THESE
Q_tau1 = M2Q(tau1*jhat,omega3*jhat);
Q_tau2 = M2Q(tau2*jhat,omega4*jhat); 
Q_tau2R= M2Q(-tau2*jhat,omega1*jhat);

%Q = Q_tau1+Q_tau2 + Q_tau2R;
Q = [0 tau1 tau2]';

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
b = simplify(A*ddq - eom); 

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

% extract x and z component
rD = [rD(1);rD(3)];
drD= [drD(1);drD(3)];
J  = [J(1,1) J(1,2); J(2,1), J(2,2)];
dJ = [dJ(1,1),dJ(1,2);dJ(2,1),dJ(2,2)];

% Compute mapping from joint space to operation space
%Lambda = inv(J*inv(Mass_Joint_Sp)*J');
%mu = Lambda*J*inv(Mass_Joint_Sp)*Corr_Joint_Sp - Lambda*dJ*dq;
%rho = Lambda*J*inv(Mass_Joint_Sp)*Grav_Joint_Sp;

matlabFunction(A,'file',['../AutoDerived/A_foot'],'vars',{z p});
matlabFunction(b,'file',['../AutoDerived/b_foot'],'vars',{z u p});
matlabFunction(E,'file',['../AutoDerived/energy_foot' ],'vars',{z p});
matlabFunction(rD,'file',['../AutoDerived/position_foot'],'vars',{z p});
matlabFunction(drD,'file',['../AutoDerived/velocity_foot'],'vars',{z p});
matlabFunction(J ,'file',['../AutoDerived/jacobian_foot'],'vars',{z p});
matlabFunction(dJ ,'file',['../AutoDerived/jacobian_dot_foot'],'vars',{z p});

matlabFunction(Mass_Joint_Sp, 'file', '../AutoDerived/Mass_matrix_foot', 'vars', {z,p});
matlabFunction(Grav_Joint_Sp ,'file', ['../AutoDerived/Grav_foot'] ,'vars',{z p});
matlabFunction(Corr_Joint_Sp ,'file', ['../AutoDerived/Corr_foot']     ,'vars',{z p});
matlabFunction(keypoints,'file',['../AutoDerived/keypoints_foot'],'vars',{z p});

% matlabFunction(Lambda , 'file', ['Lambda_leg'], 'vars', {z p});
% matlabFunction(mu , 'file', ['mu_leg'], 'vars', {z p});
% matlabFunction(rho , 'file', ['rho_leg'], 'vars', {z p});
