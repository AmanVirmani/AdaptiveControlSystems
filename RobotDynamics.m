function [q,qD,IC] = RobotDynamics(t,T,u,IC)

% Define constants
m1 = 50; m2 = 50; % kg
l1 = 1; l2 = 1;   % meter
b1 = 0.5; b2 = 0.5; % COM distance (meter)
I1 = (1/3)*(m1)*(l1^2); % Inertia
I2 = (1/3)*(m2)*(l2^2); % Inertia
g = -9.8; % Gravity Acceleration
t0 = 0 ; tf = 1;
tspan = [t0 tf];

% Define initial values theta1, theta1_dot, theta2, theta2_dot

q = zeros(2,1);
qD = zeros(2,1);

tau1 = u(1);
tau2 = u(2);

d11 = @(S) (m1*b1^2)+m2*(l1^2+b2^2+2*l1*b2*cos(S(3)))+I1+I2;
d12 = @(S) m2*(b2^2+l1*b2*cos(S(3)))+I2;
d21 = @(S) m2*(b2^2+l1*b2*cos(S(3)))+I2;
d22 = m2*b2^2+I2;

% Define Christoffel Symbols

c121 = @(S) -1*l1*b2*m2*sin(S(3));
c211 = @(S) -1*l1*b2*m2*sin(S(3));
c221 = @(S) -1*l1*b2*m2*sin(S(3));
c112 = @(S) l1*b2*m2*sin(S(3));

% Define gravity terms

g1 = @(S) (m1*b1+m2*l1)*g*cos(S(1)) + m2*b2*g*cos(S(1)+S(3));
g2 = @(S)  m2*b2*g*cos(S(1)+S(3));

% Define necessary coefficients

theta1_katsayi = @(S) (d12(S)*d22)/(d12(S)*d21(S)-d11(S)*d22);
theta2_katsayi = @(S) (d11(S)*d21(S))/(d11(S)*d22-d12(S)*d21(S));

% Define terms of theta1

theta1_1 =  @(S) ((-c112(S)/d22)*(S(2)^2));
theta1_2 =  @(S) ((c221(S)/d12(S))*(S(4)^2));
theta1_3 =  @(S) (((c121(S)+c211(S))/d12(S))*S(2)*S(4));
theta1_4 =  @(S) (g1(S)/d12(S));
theta1_5 =  @(S) (-g2(S)/d22);
theta1_6 =  @(S) (-tau1/d12(S));
theta1_7 =  (tau2/d22);

% Define terms of theta2

theta2_1 =  @(S) ((-c112(S)/d21(S))*(S(2)^2));
theta2_2 =  @(S) ((c221(S)/d11(S))*(S(4)^2));
theta2_3 =  @(S) (((c121(S)+c211(S))/d11(S))*S(2)*S(4));
theta2_4 =  @(S) (g1(S)/d11(S));
theta2_5 =  @(S) (-g2(S)/d21(S));
theta2_6 =  @(S) (-tau1/d11(S));
theta2_7 =  @(S) (tau2/d21(S));

% Define system of first order differential equations

SDOT = @(time,S) ...
    [S(2);
    theta1_katsayi(S)*(theta1_1(S)+theta1_2(S)+theta1_3(S)+theta1_4(S)+theta1_5(S)+theta1_6(S)+theta1_7);
    S(4);
    theta2_katsayi(S)*(theta2_1(S)+theta2_2(S)+theta2_3(S)+theta2_4(S)+theta2_5(S)+theta2_6(S)+theta2_7(S))];

% Numerical Integration by means of 'ODE45' Runge Kutta Method
[time state_values] = ode45 (SDOT, tspan, IC);

q(1) = state_values(end,1);
qD(1) = state_values(end,2);
q(2) = state_values(end,3);
qD(2) = state_values(end,4);

IC = [q(1),qD(1),q(2),qD(2)];

end