function p = parameters()
% Defines fixed paramters used in the simulation
mp = .00783;            % pendulum mass           
mb = 1.55;              % body mass
m1 =.0393 + .4;         % thigh mass      
m2 =.0368 + .4;         % leg mass

Ip = 9.25 * 10^-6;      % Inertia of pendulum
Ib = 22.176 * 10^-6;    % Inertia of body
I1 = 25.1 * 10^-6;      % Inertia of hip
I2 = 25.1 * 10^-6;      % Inertia of knee

l_OA=.15;               % length from origin to end of pendulum
l_AB=.10;               % end of pendulum to hip joint
l_BC=.085;              % hip joint to knee joint
l_CD=.087;              % knee joint to end effector
l_Omp=l_OA/2;           % origin to CM pendulum
l_Amb=0;                % end of pendulum to CM body
l_Bm1=l_BC/2;           % hip joint to CM thigh link
l_Cm2=l_CD/2;           % knee joint to CM leg link

N = 18.75;              % gear ratio motor
Ir = 0.0035/N^2*10^-6;        % motor inertia
g = 9.81;               % gravity

nz = 3;                 % number of generalized coordinates

%% Parameter vector
p   = [l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1 I2 Ir N g nz]';
end