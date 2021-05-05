% 20190348 Jungill Kang
addpath('..\mr')

%% Indy 7 inverse kinematics
% L1 : 449.5, L2 : 265.5, L3 : 228, H1 : 223, H2 : 84.5, W1 : 109.3, W2 : 31.1, W3 : 74.2, W4 : 114.3, W5 : 68.7
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

% q
% spatial frame
q1_s = [0 0 0]';
q2_s = [-W1 0 H1]';
q3_s = [-W1+W2 0 H1+L1]';
q4_s = [-W1+W2+W3 0 H1+L1+L2]';
q5_s = [-W1+W2+W3-W4 0 H1+L1+L2+H2]';
q6_s = [-W1+W2+W3-W4-W5 0 H1+L1+L2+H2+L3]';
% body frame
q1_b = [W5+W4-W3-W2+W1 0 -L3-H2-L2-L1-H1]';
q2_b = [W5+W4-W3-W2 0 -L3-H2-L2-L1]';
q3_b = [W5+W4-W3 0 -L3-H2-L2]';
q4_b = [W5+W4 0 -L3-H2]';
q5_b = [W5 0 -L3]';
q6_b = [0 0 0]';

% w
% spatial frame
w1_s = [0 0 1]';
w2_s = [-1 0 0]';
w3_s = [-1 0 0]';
w4_s = [0 0 1]';
w5_s = [-1 0 0]';
w6_s = [0 0 1]';
% body frame
w1_b = [0 0 1]';
w2_b = [-1 0 0]';
w3_b = [-1 0 0]';
w4_b = [0 0 1]';
w5_b = [-1 0 0]';
w6_b = [0 0 1]';

% v
% spatial frame
v1_s = VecToso3(q1_s) * w1_s;
v2_s = VecToso3(q2_s) * w2_s;
v3_s = VecToso3(q3_s) * w3_s;
v4_s = VecToso3(q4_s) * w4_s;
v5_s = VecToso3(q5_s) * w5_s;
v6_s = VecToso3(q6_s) * w6_s;
% body frame
v1_b = VecToso3(q1_b) * w1_b;
v2_b = VecToso3(q2_b) * w2_b;
v3_b = VecToso3(q3_b) * w3_b;
v4_b = VecToso3(q4_b) * w4_b;
v5_b = VecToso3(q5_b) * w5_b;
v6_b = VecToso3(q6_b) * w6_b;

% vector screw for each axis
% spatial frame
s1_s = [w1_s; v1_s];
s2_s = [w2_s; v2_s];
s3_s = [w3_s; v3_s];
s4_s = [w4_s; v4_s];
s5_s = [w5_s; v5_s];
s6_s = [w6_s; v6_s];
% body frame
s1_b = [w1_b; v1_b];
s2_b = [w2_b; v2_b];
s3_b = [w3_b; v3_b];
s4_b = [w4_b; v4_b];
s5_b = [w5_b; v5_b];
s6_b = [w6_b; v6_b];

% se3 screw for each axis
% spatial frame
s1_se3_s = VecTose3(s1_s);
s2_se3_s = VecTose3(s2_s);
s3_se3_s = VecTose3(s3_s);
s4_se3_s = VecTose3(s4_s);
s5_se3_s = VecTose3(s5_s);
s6_se3_s = VecTose3(s6_s);
% body frame
s1_se3_b = VecTose3(s1_b);
s2_se3_b = VecTose3(s2_b);
s3_se3_b = VecTose3(s3_b);
s4_se3_b = VecTose3(s4_b);
s5_se3_b = VecTose3(s5_b);
s6_se3_b = VecTose3(s6_b);

SList = [s1_s s2_s s3_s s4_s s5_s s6_s];
BList = [s1_b s2_b s3_b s4_b s5_b s6_b];

% initial matrix
M = [
    1 0 0 -W1+W2+W3-W4-W5;
    0 1 0 0;
    0 0 1 H1+L1+L2+H2+L3;
    0 0 0 1];

% random theta
% body frame
theta1_b = pi/4;
theta2_b = pi/3;
theta3_b = 0;
theta4_b = pi/5;
theta5_b = pi/2;
theta6_b = pi;
thetalist = [theta1_b theta2_b theta3_b theta4_b theta5_b theta6_b]';

% use FKinBody()
Tdesired = FKinBody(M, BList, thetalist);

% guess for IK
thetaini = [0 0 0 0 0 0]';

[IKthetas, success_s] = IKinSpace(SList, M, Tdesired, thetaini, 0.00001, 0.00001);
[IKthetab, success_b] = IKinBody(BList, M, Tdesired, thetaini, 0.00001, 0.00001);

Tchecks = FKinSpace(M, SList, IKthetas);
Tcheckb = FKinBody(M, BList, IKthetab);

thetalist, IKthetas, IKthetab, success_s, success_b, Tdesired, Tchecks, Tcheckb 

