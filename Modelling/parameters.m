function p = parameters()
% Defines fixed paramters used in the simulation
mp =.0869; %.145;               % pendulum mass           
mb =.0436 + 2*0.20;    % body mass + 2 motors
m1 =.047 + .20 + 0.014; % thigh mass + 1 motor + clamp
m2 =.02274 + .014;      % leg mass + clamp
m3 =.01756 + .014;      % arm mass 

Ip = 67.2 * 10^-5;      % Inertia of pendulum
Ib = 13.1 * 10^-4;    % Inertia of body
I1 = 13.3 * 10^-4;       % Inertia of hip
I2 = 19.2 * 10^-5;      % Inertia of knee
I3 = 22.1 * 10^-5;      % Inertia of arm

l_OA=.152;              % length from origin to end of pendulum
l_AB=.083;              % end of pendulum to hip joint
l_BC=.082;              % length of thigh link
l_CD=.088;              % length of leg link
l_AE=.096;              % length of arm link
l_Omp=l_OA/2;           % origin to CM pendulum
l_Amb=0.0461;           % end of pendulum to CM body
l_Bm1=0.0661;           % hip joint to CM thigh link
l_Cm2=0.0537; %l_CD;           % knee joint to CM leg link
l_Am3=0.0649; %l_AE;           % shoulder joint to CM of arm link

N = 18.75;              % gear ratio motor
Ir = 0.0035/N^2*10^-6;  % motor inertia
g = 9.81;               % gravity

nz = 4;                 % number of generalized coordinates

%% Parameter vector
% p   = [l_OA l_AB l_BC l_CD l_Omp l_Amb l_Bm1 l_Cm2 mp mb m1 m2 Ip Ib I1
% I2 Ir N g nz]'; % used for the case without arm
p = [l_OA l_AB l_BC l_CD l_AE l_Omp l_Amb l_Bm1 l_Cm2 l_Am3 mp mb m1 m2 m3 Ip Ib I1 I2 I3 Ir N g nz]';

end