% 20190348 강준길
addpath('C:\ws\git\ModernRobotics\packages\MATLAB\mr')

%% p21
% L1 and L2 as 10
L1 = 10;
L2 = 10;

% q
% spatial frame
q1_s = [0 0 0]';
q2_s = [0 0 0]';
q3_s = [0 0 0]';
q4_s = [0 0 0]';
q5_s = [0 L1 0]';
q6_s = [0 L1+L2 0]';
% body frame
q1_b = [0 -L1-L2 0]';
q2_b = [0 -L1-L2 0]';
q3_b = [0 0 0]';
q4_b = [0 0 0]';
q5_b = [0 -L2 0]';
q6_b = [0 0 0]';

% w
% spatial frame
w1_s = [0 0 1]';
w2_s = [1 0 0]';
w3_s = [0 0 0]';
w4_s = [0 1 0]';
w5_s = [1 0 0]';
w6_s = [0 1 0]';
% body frame
w1_b = [0 0 1]';
w2_b = [1 0 0]';
w3_b = [0 0 0]';
w4_b = [0 1 0]';
w5_b = [1 0 0]';
w6_b = [0 1 0]';

% v
% spatial frame
v1_s = VecToso3(q1_s) * w1_s;
v2_s = VecToso3(q2_s) * w2_s;
v3_s = [0 1 0]';
v4_s = VecToso3(q4_s) * w4_s;
v5_s = VecToso3(q5_s) * w5_s;
v6_s = VecToso3(q6_s) * w6_s;
% body frame
v1_b = VecToso3(q1_b) * w1_b;
v2_b = VecToso3(q2_b) * w2_b;
v3_b = [0 1 0]';
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
s1_se3_s = VecTose3(s1_s);
s2_se3_s = VecTose3(s2_s);
s3_se3_s = VecTose3(s3_s);
s4_se3_s = VecTose3(s4_s);
s5_se3_s = VecTose3(s5_s);
s6_se3_s = VecTose3(s6_s);

screw_s = [s1_s s2_s s3_s s4_s s5_s s6_s];

% initial matrix
M = [
    1 0 0 0;
    0 1 0 L1+L2;
    0 0 1 0;
    0 0 0 1];

% random theta
theta1_s = 0;
theta2_s = 0;
theta3_s = 0;
theta4_s = 0;
theta5_s = pi/2;
theta6_s = 0;

% transformation matrix for se3 matrix
T1_s = MatrixExp6(s1_se3_s * theta1_s);
T2_s = MatrixExp6(s2_se3_s * theta2_s);
T3_s = MatrixExp6(s3_se3_s * theta3_s);
T4_s = MatrixExp6(s4_se3_s * theta4_s);
T5_s = MatrixExp6(s5_se3_s * theta5_s);
T6_s = MatrixExp6(s6_se3_s * theta6_s);

% transformed Transformation matrix
T_final_s = T1_s * T2_s * T3_s * T4_s * T5_s * T6_s * M;

disp("p21");
disp("For spatial theta( 1 2 3 4 5 6 )=>( 0 0 0 0 pi/2 0 )");
disp("T_final in spatial twist");
disp(T_final_s);
disp("For body theta( 1 2 3 4 5 6 )=>( 0 0 0 0 pi/2 0 )");
disp("T_final in body twist");
disp(T_final_s);

%% p22
screw;
M;
theta;
T_final_s;

%% p23
screw;
M;
theta;
T_final_s;

%% p26
screw;
M;
theta;
T_final_s;
%% p30
screw;
M;
theta;
T_final_s;

%% p31
screw;
M;
theta;
T_final_s;

%% p33
screw;
M;
theta;
T_final_s;

%% p34
screw;
M;
theta;
T_final_s;

%% p36
screw;
M;
theta;
T_final_s;

%% p37
screw;
M;
theta;
T_final_s;
