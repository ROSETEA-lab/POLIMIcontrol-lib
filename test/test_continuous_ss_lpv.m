clear all;

% Create the parameter vector
Ts = 1e-4;
t = (0:Ts:15)';

num_param = max(round(5*rand(1)),1);
for k=1:num_param
    param(:,k) = rand(1)*sin(100*rand(1)*t);
end

% Create the system matrices
n = max(round(25*rand(1)),1);
m = max(round(5*rand(1)),1);
p = max(round(5*rand(1)),1);

eig_v = 10*(rand(1,n)-1);
if not(isempty(find(eig_v == 0)))
    eig_v(find(eig_v == 0)) = -1;
end

A0 = diag(eig_v);
T = rand(n,n);

A0  = T*A0*inv(T);
A1  = 1e-10*round(2*(rand(n,n)-0.5));
for k=1:num_param
    Ap(:,:,k) = 1e-3*round(2*(rand(n,n)-0.5));
end

B0  = rand(n,m);
B1  = 1e-3*round(2*(rand(n,m)-0.5));
for k=1:num_param
    Bp(:,:,k) = 2e-3*(rand(n,m)-0.5);
end

C0 = rand(p,n);
C1 = 1e-3*round(2*(rand(p,n)-0.5));
for k=1:num_param
    Cp(:,:,k) = 2e-3*(rand(p,n)-0.5);
end

D0 = rand(p,m);
D1 = 1e-3*round(2*(rand(p,m)-0.5));
for k=1:num_param
    Dp(:,:,k) = 2e-3*(rand(p,m)-0.5);
end

% Create the input signal
in = zeros(length(t),m);
for k=1:m
    in(:,k) = 20*(rand(1,1)-0.5)+chirp(t,0,t(end),500)';
end

% Simulate filter
initial_state = rand(n,1);
sim_out = sim('test_continuous_ss_lpv_model');

output = sim_out.y;
state  = sim_out.x;
