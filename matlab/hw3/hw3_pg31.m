% 20190348 Jungill Kang
addpath('..\mr')

%% p31
% L1 : 0.550, L2 : 0.3, L3 : 0.06, W1 : 0.045
L1 = 0.550;
L2 = 0.3;
L3 = 0.06;
W1 = 0.045;

% q
% spatial frame
q1_s = [0 0 0]';
q2_s = [0 0 0]';
q3_s = [0 0 0]';
q4_s = [W1 0 L1]';
q5_s = [0 0 0]';
q6_s = [0 0 L1+L2]';
q7_s = [0 0 0]';
% body frame
q1_b = [0 0 0]';
q2_b = [0 0 -L1-L2-L3]';
q3_b = [0 0 0]';
q4_b = [W1 0 -L2-L3]';
q5_b = [0 0 0]';
q6_b = [0 0 -L3]';
q7_b = [0 0 0]';

% w
% spatial frame
w1_s = [0 0 1]';
w2_s = [0 1 0]';
w3_s = [0 0 1]';
w4_s = [0 1 0]';
w5_s = [0 0 1]';
w6_s = [0 1 0]';
w7_s = [0 0 1]';
% body frame
w1_b = [0 0 1]';
w2_b = [0 1 0]';
w3_b = [0 0 1]';
w4_b = [0 1 0]';
w5_b = [0 0 1]';
w6_b = [0 1 0]';
w7_b = [0 0 1]';

% v
% spatial frame
v1_s = VecToso3(q1_s) * w1_s;
v2_s = VecToso3(q2_s) * w2_s;
v3_s = VecToso3(q3_s) * w3_s;
v4_s = VecToso3(q4_s) * w4_s;
v5_s = VecToso3(q5_s) * w5_s;
v6_s = VecToso3(q6_s) * w6_s;
v7_s = VecToso3(q7_s) * w7_s;
% body frame
v1_b = VecToso3(q1_b) * w1_b;
v2_b = VecToso3(q2_b) * w2_b;
v3_b = VecToso3(q3_b) * w3_b;
v4_b = VecToso3(q4_b) * w4_b;
v5_b = VecToso3(q5_b) * w5_b;
v6_b = VecToso3(q6_b) * w6_b;
v7_b = VecToso3(q7_b) * w7_b;

% vector screw for each axis
% spatial frame
s1_s = [w1_s; v1_s];
s2_s = [w2_s; v2_s];
s3_s = [w3_s; v3_s];
s4_s = [w4_s; v4_s];
s5_s = [w5_s; v5_s];
s6_s = [w6_s; v6_s];
s7_s = [w7_s; v7_s];
% body frame
s1_b = [w1_b; v1_b];
s2_b = [w2_b; v2_b];
s3_b = [w3_b; v3_b];
s4_b = [w4_b; v4_b];
s5_b = [w5_b; v5_b];
s6_b = [w6_b; v6_b];
s7_b = [w7_b; v7_b];

% se3 screw for each axis
% spatial frame
s1_se3_s = VecTose3(s1_s);
s2_se3_s = VecTose3(s2_s);
s3_se3_s = VecTose3(s3_s);
s4_se3_s = VecTose3(s4_s);
s5_se3_s = VecTose3(s5_s);
s6_se3_s = VecTose3(s6_s);
s7_se3_s = VecTose3(s7_s);
% body frame
s1_se3_b = VecTose3(s1_b);
s2_se3_b = VecTose3(s2_b);
s3_se3_b = VecTose3(s3_b);
s4_se3_b = VecTose3(s4_b);
s5_se3_b = VecTose3(s5_b);
s6_se3_b = VecTose3(s6_b);
s7_se3_b = VecTose3(s7_b);

screw_s = [s1_s s2_s s3_s s4_s s5_s s6_s s7_s];
screw_b = [s1_b s2_b s3_b s4_b s5_b s6_b s7_b];

% initial matrix
M = [
    1 0 0 0;
    0 1 0 0;
    0 0 1 L1+L2+L3;
    0 0 0 1];

% random theta
% spatial frame
theta1_s = 0;
theta2_s = pi/4;
theta3_s = 0;
theta4_s = -pi/4;
theta5_s = 0;
theta6_s = -pi/2;
theta7_s = 0;
% body frame
theta1_b = 0;
theta2_b = pi/4;
theta3_b = 0;
theta4_b = -pi/4;
theta5_b = 0;
theta6_b = -pi/2;
theta7_b = 0;

% transformation matrix for se3 matrix
% spatial frame
T1_s = MatrixExp6(s1_se3_s * theta1_s);
T2_s = MatrixExp6(s2_se3_s * theta2_s);
T3_s = MatrixExp6(s3_se3_s * theta3_s);
T4_s = MatrixExp6(s4_se3_s * theta4_s);
T5_s = MatrixExp6(s5_se3_s * theta5_s);
T6_s = MatrixExp6(s6_se3_s * theta6_s);
T7_s = MatrixExp6(s7_se3_s * theta7_s);
% body frame
T1_b = MatrixExp6(s1_se3_b * theta1_b);
T2_b = MatrixExp6(s2_se3_b * theta2_b);
T3_b = MatrixExp6(s3_se3_b * theta3_b);
T4_b = MatrixExp6(s4_se3_b * theta4_b);
T5_b = MatrixExp6(s5_se3_b * theta5_b);
T6_b = MatrixExp6(s6_se3_b * theta6_b);
T7_b = MatrixExp6(s6_se3_b * theta7_b);

% transformed Transformation matrix
% spatial frame
T_final_s = T1_s * T2_s * T3_s * T4_s * T5_s * T6_s * T7_s * M;
% body frame
T_final_b = M * T1_b * T2_b * T3_b * T4_b * T5_b * T6_b * T7_b;

disp("-------------------");
disp("p31");
disp("For spatial theta( 1 2 3 4 5 6 7 )=>( 0 pi/4 0 -pi/4 0 -pi/2 0 )");
disp("T_final in spatial twist");
disp(T_final_s);
disp("For body theta( 1 2 3 4 5 6 7 )=>( 0 pi/4 0 -pi/4 0 -pi/2 0 )");
disp("T_final in body twist");
disp(T_final_b);

% checking purpose
AdjM = Adjoint(M);
disp("screw_s");
disp(screw_s);
disp("AdjM(screw_b)");
disp(AdjM * screw_b);