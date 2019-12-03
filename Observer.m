function [EqD,Eq] = Observer(T,q,qdDD)

% Observer Hyperparameters
ld = 250;

% Static Variables
persistent Eq_prev EqD_prev w_prev wD_prev
n = length(q);
if isempty(Eq_prev)
    Eq_prev = zeros(n,1);
end
if isempty(EqD_prev)
    EqD_prev = zeros(n,1);
end
if isempty(w_prev)
    w_prev = zeros(n,1);
end
if isempty(wD_prev)
    wD_prev = zeros(n,1);
end

L2 = diag([0.1 0.1]); % Controller Hyperparameter
Ld = ld*eye(2) + L2;
Lp = ld*L2;

wD = qdDD + Ld*(q - Eq_prev);
w = trapz(T,[wD_prev'; wD'])' + w_prev;

EqD = Lp*(q - Eq_prev) + w;
Eq = Eq_prev; %% of prev cycle; check if it is correct

Eq_prev = trapz(T,[EqD_prev'; EqD'])' + Eq_prev;
EqD_prev = EqD;
w_prev = w;
wD_prev = wD;

end