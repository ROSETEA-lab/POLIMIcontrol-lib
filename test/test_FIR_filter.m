clear all;

% Create the input signal
t = 0:0.001:60;
in = chirp(t,0,t(end),500);
t = 0:1:length(in)-1;

% Create filter coefficients
order = max(round(25*rand(1)),1);
coeff = 20.0*(rand(1,order)-0.5);

% Create initial filter state
initial_state = 50.0*(rand(1,order-1)-0.5);

% Simulate filter
out = filter(coeff,[1, zeros(1,length(coeff)-1)],in',initial_state);
