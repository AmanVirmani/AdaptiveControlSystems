function y = MLP(t,T,qd,qdD,S)
% This NN design has 2 hidden layers (4 neurons + 3 neurons consecutively)
% Search specialized direct neural control for more info

% Learning Hyperparametrs
learningRate3 = (1 - exp(-0.5*t))*diag([2800 2200]);
learningRate2 = 1500*(1 - exp(-0.5*t))*eye(3);
learningRate1 = 20*eye(4);
B1 = 0.165*(1 - exp(-0.5*t))*ones(4,2);
B2 = 0.165*(1 - exp(-0.5*t))*ones(3,2);
sigma = [0.03 0.03 3e-5];
W1_0 = ones(5,4);
W2_0 = ones(5,3);
W3_0 = ones(4,2);

% Static Variables
persistent W1_prev W1D_prev W2_prev W2D_prev W3_prev W3D_prev
if isempty(W1_prev)
    W1_prev = W1_0;
end
if isempty(W2_prev)
    W2_prev = W2_0;
end
if isempty(W3_prev)
    W3_prev = W3_0;
end
if isempty(W1D_prev)
    W1D_prev = zeros(5,4);
end
if isempty(W2D_prev)
    W2D_prev = zeros(5,3);
end
if isempty(W3D_prev)
    W3D_prev = zeros(4,2);
end

% Forward Pass
a0 = [qd; qdD]; % construct the NN input
a1 = Sigmoid(W1_prev'*[a0; 1]);
a2 = Sigmoid(W2_prev'*[a1; 1]);
a3 = W3_prev'*[a2; 1];

y = a3;

% Backpropagation
W3D = -(sigma(3)*(W3_prev-W3_0) + [a2; 1]*S')*learningRate3;
W2D = -(sigma(2)*(W2_prev-W2_0) + [a1; 1]*((W2_prev-W2_0)'*[a1; 1] - B2*S)')*learningRate2;
W1D = -(sigma(1)*(W1_prev-W1_0) + [a0; 1]*(( W1_prev-W1_0)'*[a0; 1] - B1*S)')*learningRate1;

% Parameters Update
W1 = IntegrateMatrix(T,W1D,W1D_prev) + W1_prev;
W2 = IntegrateMatrix(T,W2D,W2D_prev) + W2_prev;
W3 = IntegrateMatrix(T,W3D,W3D_prev) + W3_prev;

% Buffer Previous Values
W1_prev = W1;
W1D_prev = W1D;
W2_prev = W2;
W2D_prev = W2D;
W3_prev = W3;
W3D_prev = W3D;

end