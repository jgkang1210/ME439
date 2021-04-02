% 20190348 Jungill Kang
addpath('..\mr')

%% p58

% initial condition
L1 = 449.5;
L2 = 265.5;
L3 = 228;
H1 = 223;
H2 = 84.5;
W1 = 109.3;
W2 = 31.1;
W3 = 74.2;
W4 = 114.3;
W5 = 68.7;

% joint value
% change theta 5
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = pi/2;
theta6 = 0;

% D-H parameter - Rot(z, theta)
theta1 = theta1 - pi/2;
theta2 = theta2 + pi/2;
theta3 = theta3 - pi/2;
theta4 = theta4 + 0;
theta5 = theta5 + 0;
theta6 = theta6 + pi/2;

% D-H parameter - Trans(z, d)
d1 = H1;
d2 = W1- W2;
d3 = -W3;
d4 = L2 + H2;
d5 = W4 + W5;
d6 = L3;

% D-H parameter - Trans(x, a)
a1 = 0;
a2 = L1;
a3 = 0;
a4 = 0;
a5 = 0;
a6 = 0;

% D-H parameter - Rot(x, alpha)
alpha1 = pi/2;
alpha2 = 0;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = 0;

% SE3 matrix for each D-H parameters
% DH function -- same path
T1 = DH(theta1, d1, a1, alpha1);
T2 = DH(theta2, d2, a2, alpha2);
T3 = DH(theta3, d3, a3, alpha3);
T4 = DH(theta4, d4, a4, alpha4);
T5 = DH(theta5, d5, a5, alpha5);
T6 = DH(theta6, d6, a6, alpha6);

% result
T = T1*T2*T3*T4*T5*T6;

disp("-------------------");
disp("For theta( 1 2 3 4 5 6 )=>( 0 0 0 0 pi/2 0 )");
disp("T_final");
disp(T);