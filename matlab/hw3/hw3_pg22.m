% 20190348 Jungill Kang
addpath('..\mr')

%% p22
% L1 : 10, L2 : 9, L3 = 3
L1 = 10;
L2 = 9;
L3 = 3;

% q
% spatial frame
q1_s = [0 0 0]';
q2_s = [L1 0 0]';
q3_s = [L1+L2 0 0]';
q4_s = [L1+L2 0 0]';

% body frame
q1_b = [0 L1+L2 0]';
q2_b = [0 L2 0]';
q3_b = [0 0 0]';
q4_b = [0 0 0]';


% w
% spatial frame
w1_s = [0 0 1]';
w2_s = [0 0 1]';
w3_s = [0 0 1]';
w4_s = [0 0 -1]';

% body frame
w1_b = [0 0 -1]';
w2_b = [0 0 -1]';
w3_b = [0 0 -1]';
w4_b = [0 0 1]';


% v
% spatial frame
v1_s = VecToso3(q1_s) * w1_s;
v2_s = VecToso3(q2_s) * w2_s;
v3_s = VecToso3(q3_s) * w3_s;
v4_s = VecToso3(q4_s) * w4_s;

% body frame
v1_b = VecToso3(q1_b) * w1_b;
v2_b = VecToso3(q2_b) * w2_b;
v3_b = VecToso3(q3_b) * w3_b;
v4_b = VecToso3(q4_b) * w4_b;

% vector screw for each axis
% spatial frame
s1_s = [w1_s; v1_s];
s2_s = [w2_s; v2_s];
s3_s = [w3_s; v3_s];
s4_s = [w4_s; v4_s];

% body frame
s1_b = [w1_b; v1_b];
s2_b = [w2_b; v2_b];
s3_b = [w3_b; v3_b];
s4_b = [w4_b; v4_b];

% se3 screw for each axis
% spatial frame
s1_se3_s = VecTose3(s1_s);
s2_se3_s = VecTose3(s2_s);
s3_se3_s = VecTose3(s3_s);
s4_se3_s = VecTose3(s4_s);

% body frame
s1_se3_b = VecTose3(s1_b);
s2_se3_b = VecTose3(s2_b);
s3_se3_b = VecTose3(s3_b);
s4_se3_b = VecTose3(s4_b);

screw_s = [s1_s s2_s s3_s s4_s];
screw_b = [s1_b s2_b s3_b s4_b];

% initial matrix
M = [
    0 -1 0 L1+L2;
    -1 0 0 0;
    0 0 -1 -L3;
    0 0 0 1];

% random theta
% spatial frame
theta1_s = 0;
theta2_s = pi/2;
theta3_s = 0;
theta4_s = 0;

% body frame
theta1_b = 0;
theta2_b = pi/2;
theta3_b = 0;
theta4_b = 0;


% transformation matrix for se3 matrix
% spatial frame
T1_s = MatrixExp6(s1_se3_s * theta1_s);
T2_s = MatrixExp6(s2_se3_s * theta2_s);
T3_s = MatrixExp6(s3_se3_s * theta3_s);
T4_s = MatrixExp6(s4_se3_s * theta4_s);

% body frame
T1_b = MatrixExp6(s1_se3_b * theta1_b);
T2_b = MatrixExp6(s2_se3_b * theta2_b);
T3_b = MatrixExp6(s3_se3_b * theta3_b);
T4_b = MatrixExp6(s4_se3_b * theta4_b);


% transformed Transformation matrix
% spatial frame
T_final_s = T1_s * T2_s * T3_s * T4_s * M;
% body frame
T_final_b = M * T1_b * T2_b * T3_b * T4_b;

disp("-------------------");
disp("p22");
disp("For spatial theta( 1 2 3 4 )=>( 0 pi/2 0 0 )");
disp("T_final in spatial twist");
disp(T_final_s);
disp("For body theta( 1 2 3 4 )=>( 0 pi/2 0 0 )");
disp("T_final in body twist");
disp(T_final_b);

% checking purpose
AdjM = Adjoint(M);
disp("screw_s");
disp(screw_s);
disp("AdjM(screw_b)");
disp(AdjM * screw_b);