clear; clc;
addpath(genpath('casadi'))
import casadi.*

%%
opti = casadi.Opti()
p = opti.parameter(19, 1);
q = opti.variable(4, 1);

%%
test = A_project(q, p)


function f = dynamics(x, u, p)

end