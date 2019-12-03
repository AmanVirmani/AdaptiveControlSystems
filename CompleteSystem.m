clear all; clc

% Number of DOFs of the Robot
n = 2;

% Sampling Interval
T = 0.005; % s

% Set up the System
qd = [1; 3];
qdD = zeros(n,1);
qdDD = zeros(n,1);
q = [0.5; 2.0];
qD = zeros(n,1);
Eq = q;
EqD = qD;
IC = [q(1),qD(1),q(2),qD(2)];
u = Controller(0,T,qd,qdD,qdDD,q,qD,Eq,EqD);

% Run the System
runningTime = 1; % s
for t = T:T:runningTime
    % Reference Trajectory
    qd = [cos(1.5*t) 3*cos(t)]';
    qdD = [-1.5*sin(1.5*t) -3*sin(t)]';
    qdDD = [-2.25*cos(1.5*t) -3*cos(t)]';
    
    % Execute System Logic
    [q,qD,IC] = RobotDynamics(t,T,u,IC);
    [EqD,Eq] = Observer(T,q,qdDD);
    u = Controller(t,T,qd,qdD,qdDD,q,qD,Eq,EqD);
    
    % Tracking Error
    e = q - qd
    eD = qD - qdD
end