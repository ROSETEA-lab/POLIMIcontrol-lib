clear all;

% Create the input signal
Ts   = 0.001;
Tend = 20;
t  = 0:Ts:Tend;
in = chirp(t,0,Tend,50,'linear',-90); % Sinus chirp

% Set integrator parameters
K = 50*rand(1,1);
initial_condition = 0;

% Simulate the system
simres = sim('test_discrete_derivative_model', [0 Tend]);
