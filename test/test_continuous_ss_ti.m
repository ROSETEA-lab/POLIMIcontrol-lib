clear all;

% Create the system matrices
n = max(round(25*rand(1)),1);
m = max(round(5*rand(1)),1);
p = max(round(5*rand(1)),1);

eig_v = 10*(rand(1,n)-1);
if not(isempty(find(eig_v == 0)))
    eig_v(find(eig_v == 0)) = -1;
end

A = diag(eig_v);
T = rand(n,n);

A = T*A*inv(T);
B = rand(n,m);
C = rand(p,n);
D = rand(p,m);

% Create the input signal
Ts = 1e-4;
t = (0:Ts:15)';

in = zeros(length(t),m);
for k=1:m
    in(:,k) = 20*(rand(1,1)-0.5)+chirp(t,0,t(end),500)';
end

% Simulate system
initial_state = rand(n,1);

output = lsim(ss(A,B,C,D),in,t,initial_state);
state  = lsim(ss(A,B,eye(n,n),zeros(n,m)),in,t,initial_state);
