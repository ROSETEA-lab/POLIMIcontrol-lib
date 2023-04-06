clear all;

% Create the input signal
t = 0:0.001:60;
y   = chirp(t,0,t(end),250);
ysp = chirp(t,250,t(end),500);

% Create PID coefficients
Kp = max(min(50*rand(1),25),0.1);
Ti = max(min(50*rand(1),10),0.05);
Td = max(min(50*rand(1),10),0.05);
N  = max(min(round(100*rand(1)),100),10);
b  = rand(1);
c  = rand(1);
Ts = 0.001;
uMax = 1e15; % disable anti-windup that is implemented in a different way
uMin = -uMax;
PID_param = [Kp, Ti, Td, N, b, c, Ts, uMin, uMax];

% Simulate controller
sim_out = sim('test_PIDISA_controller_model');
out = sim_out.control;
