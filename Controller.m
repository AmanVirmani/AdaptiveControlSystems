function u = Controller(t,T,qd,qdD,qdDD,q,qD,Eq,EqD)

% Controller Hyperparameters
L1 = diag([40 40]);
L2 = diag([0.1 0.1]);
Kd = diag([100 50]);
Em = 0;

qrD = qdD - L1*(q-qd);
qoD = EqD - L2*(q-Eq);
S   = 2*qD - qoD - qrD;
SP  = 2*EqD - qoD - qrD;

u = MLP(t,T,q,qD,S) - Kd*(qoD-qrD) - Em*sign(SP);
%u = MLP(t,T,qd,qdD,qdDD,S) - Kd*(qoD-qrD) - Em*sign(SP);

end