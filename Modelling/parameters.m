function p = parameters()
% Defines fixed paramters used in the simulation
mp = .00783;            % pendulum mass           
mb = 1.55;              % body mass
m1 =.0393 + .4;         % thigh mass      
m2 =.0368 + .4;         % leg mass
m3 =.0368 + .4;         % arm mass 

Ip = 9.25 * 10^-6;      % Inertia of pendulum
Ib = 22.176 * 10^-6;    % Inertia of body
I1 = 25.1 * 10^-6;      % Inertia of hip
I2 = 25.1 * 10^-6;      % Inertia of knee
I3 = 25.1 * 10^-6;      % Inertia of arm

l_OA=.15;               % length from origin to end of pendulum
l_AB=.10;               % end of pendulum to hip joint
l_BC=.085;              % length of thigh link
l_CD=.087;              % length of leg link
l_AE=.080;              % length of arm link
l_Omp=l_OA/2;           % origin to CM pendulum
l_Amb=0;                % end of pendulum to CM body
l_Bm1=l_BC/2;           % hip joint to CM thigh link
l_Cm2=l_CD/2;           % knee joint to CM leg link
l_Am3=l_AE/2;           % shoulder joint to CM of arm link

N = 18.75;              % gear ratio motor
Ir = 0.0035/N^2*10^-6;  % motor inertia
g = 9.81;               % gravity

nz = 4;                 % number of generalized coordinates

%% Parameter vector
% p   = [l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1
% I2 Ir N g nz]'; % used for the case without arm
p = [l_OA l_AB l_BC l_CD l_AE l_Omp l_Amb l_Bm1 l_Cm2 l_Am3 mp mb m1 m2 m3 Ip Ib I1 I2 I3 Ir N g nz]';

end