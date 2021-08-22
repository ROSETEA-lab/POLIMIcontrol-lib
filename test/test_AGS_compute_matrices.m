%% Compute X and Y
X = controller.Xi{1};
for k=2:length(controller.Xi)
    X = X + theta(k-1)*controller.Xi{k};
end

Y = controller.Yi{1};
for k=2:length(controller.Yi)
    Y = Y + theta(k-1)*controller.Yi{k};
end

%% Compute hat and plant matrices
% Constant term
Ak_hat = controller.Aki{1};
Bk_hat = controller.Bki{1};
Ck_hat = controller.Cki{1};
Dk_hat = controller.Dki{1};

A = controller.Ai{1};
B = controller.Bi{1};
C = controller.Ci{1};

% Theta dependent terms
for k=2:length(controller.Aki)
    Ak_hat = Ak_hat + theta(k-1)*controller.Aki{k};
    Bk_hat = Bk_hat + theta(k-1)*controller.Bki{k};
    Ck_hat = Ck_hat + theta(k-1)*controller.Cki{k};
    Dk_hat = Dk_hat + theta(k-1)*controller.Dki{k};
    
    A = A + theta(k-1)*controller.Ai{k};
    B = B + theta(k-1)*controller.Bi{k};
    C = C + theta(k-1)*controller.Ci{k};
end

%% Compute controller matrices
NMt = eye(size(X)) - X*Y;
[N,Mt] = lu(NMt);

Ak = N\(Ak_hat-X*(A-B*Dk_hat*C)*Y-Bk_hat*C*Y-X*B*Ck_hat)/Mt;
Bk = N\(Bk_hat-X*B*Dk_hat);
Ck = (Ck_hat-Dk_hat*C*Y)/Mt;
Dk = Dk_hat;
