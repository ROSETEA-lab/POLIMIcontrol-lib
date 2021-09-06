clear all;

% Create the input signal
Ts   = 0.001;
Tend = 20;
t  = 0:Ts:Tend;
in = chirp(t,0,Tend,50,'linear',-90); % Sinus chirp

% Set integrator parameters
K = 50*rand(1,1);
initial_condition = 50*rand(1,1);

% Simulate the system
open_system('test_discrete_integrator_model');

set_param('test_discrete_integrator_model/Discrete-Time Integrator','IntegratorMethod','Integration: Forward Euler');
set_param('test_discrete_integrator_model/Discrete-Time Integrator','InitialCondition',mat2str(initial_condition));
simres = sim('test_discrete_integrator_model', [0 Tend]);
out_fwEul = out;
clear out simres

set_param('test_discrete_integrator_model/Discrete-Time Integrator','IntegratorMethod','Integration: Backward Euler');
set_param('test_discrete_integrator_model/Discrete-Time Integrator','InitialCondition',mat2str(initial_condition));
simres = sim('test_discrete_integrator_model', [0 Tend]);
out_bwEul = out;
clear out simres

set_param('test_discrete_integrator_model/Discrete-Time Integrator','IntegratorMethod','Integration: Trapezoidal');
set_param('test_discrete_integrator_model/Discrete-Time Integrator','InitialCondition',mat2str(initial_condition));
simres = sim('test_discrete_integrator_model', [0 Tend]);
out_trapz = out;
clear out simres

close_system('test_discrete_integrator_model',0);
