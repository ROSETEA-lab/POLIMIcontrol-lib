clear all;

% Create the input signal
t = 0:0.001:60;
in = chirp(t,0,t(end),500,'linear',-90); % Sinus chirp
t = 0:1:length(in)-1;

% Create the numerator and denominator coefficients
np =round(15*rand(1));
nz = round(np*rand(1));
num_roots = 10.0*(rand(1,nz)-0.5);
den_roots =  1.9*(rand(1,np)-0.5);
num = poly(num_roots);
den = poly(den_roots);

% Simulate filter with zero initial conditions
out_z = filter(num,den,in');

% Create initial condition for non-zero initial condition test
initial_condition = 5*rand(1,np);
out_nz = filter(num,den,in',initial_condition);
