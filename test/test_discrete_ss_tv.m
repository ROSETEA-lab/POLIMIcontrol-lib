clear all;

% Create the system matrices
n = max(round(25*rand(1)),1);
m = max(round(5*rand(1)),1);
p = max(round(5*rand(1)),1);

A0 = diag(1.9*(rand(1,n)-0.5));
T = rand(n,n);

A0 = T*A0*inv(T);
A1 = 1e-10*round(2*(rand(n,n)-0.5));
B0 = rand(n,m);
B1 = round(2*(rand(n,m)-0.5));
C0 = rand(p,n);
C1 = round(2*(rand(p,n)-0.5));
D0 = rand(p,m);
D1 = round(2*(rand(p,m)-0.5));

% Create the input signal
t = (0:0.001:50)';
for k=1:m
    in(:,m) = chirp(t,0,t(end),500)';
end
t = (0:1:size(in,1)-1)';

% Simulate filter
initial_state = rand(n,1);
sim_out = sim('test_discrete_ss_tv_model');

output = sim_out.y;
state  = sim_out.x;
