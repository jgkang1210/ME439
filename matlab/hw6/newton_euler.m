% 20190348 Jungill Kang
% addpath('..\mr')

clear;

%% Robot's Properties

% mass (kg)
m = [11.80; 8.00; 3.00; 2.12; 2.29; 0.40];

% Moment of inertia (kg*m^2)
I1 = zeros(3,3); I1(1,1)=0.1542; I1(2,2)=0.1294; I1(3,3)=0.0600;
I2 = zeros(3,3); I2(1,1)=0.2936; I2(2,2)=0.2809; I2(3,3)=0.0362;
I3 = zeros(3,3); I3(1,1)=0.0342; I3(2,2)=0.0341; I3(3,3)=0.0045;
I4 = zeros(3,3); I4(1,1)=0.0067; I4(2,2)=0.0028; I4(3,3)=0.0062;
I5 = zeros(3,3); I5(1,1)=0.0099; I5(2,2)=0.0098; I5(3,3)=0.0027;
I6 = zeros(3,3); I6(1,1)=0.0004; I6(2,2)=0.0004; I6(3,3)=0.0006;
Ilist = cat(3, I1, I2, I3, I4, I5, I6);

% length
L1 = 449.5 * 0.001;
L2 = 265.5 * 0.001;
L3 = 228 * 0.001;
H1 = 223 * 0.001;
H2 = 84.5 * 0.001;
W1 = 109.3 * 0.001;
W2 = 31.1 * 0.001;
W3 = 74.2 * 0.001;
W4 = 114.3 * 0.001;
W5 = 68.7 * 0.001;
q1 = 111.5 * 0.001;
q2 = 70 * 0.001;
q3 = 220 * 0.001;
q4 = 150 * 0.001;
q5 = 192 * 0.001;
q6 = 210 * 0.001;

% vector from origin of frame {i-1} to center of mass Ci, from body cord
p0_c1 = [0; 0; q1];
p1_c2 = [q3; 0; q2];
p2_c3 = [q4; 0; -W3];
p3_c4 = [0; 0; H2];
p4_c5 = [0; 0; W5];
p5_c6 = [0; 0; -q5+q6];
pi_1_c_i = cat(3, p0_c1, p1_c2, p2_c3, p3_c4, p4_c5, p5_c6);

% vector from origion of frame {i} to center of mass Ci, from body cord
p1_c1 = [W1; 0; -H1+q1];
p2_c2 = [-L1+q3; 0; W2+q2];
p3_c3 = [-L2+q4; 0; 0];
p4_c4 = [W4; 0; 0];
p5_c5 = [-q5; 0; 0];
p6_c6 = [0; 0; -L3+q6];
pi_c_i = cat(3, p1_c1, p2_c2, p3_c3, p4_c4, p5_c5, p6_c6);

% vector from origin of frame {i-1} to origin of frame {i}, from body cord
p0_1 = [-W1; 0; H1];
p1_2 = [L1; 0; -W2];
p2_3 = [L2; 0; -W3];
p3_4 = [-W4; 0; H2];
p4_5 = [q5; 0; W5];
p5_6 = [0; 0; -q5+L3];
pi_1_p_i = cat(3, p0_1, p1_2, p2_3, p3_4, p4_5, p5_6);

%% Newton-Euler formulation

% configuration
thetalist = [0; pi/2; 0; 0; pi/2; 0]; % indy 7 : n = 7
% angular velocity of each joints
dthetalist = [0; 0; 0; 0; 0; 0];
% angular acceleration of each joints
ddthetalist = [0; 0; 0; 0; 0; 0];

g = [0; 0; -9.8];

Ftip = [0; 0; 0];
Mtib = [0; 0; 0];

taulist = NE(thetalist, dthetalist, ddthetalist, g, ...
            Mtib, Ftip, m, Ilist, pi_1_c_i, pi_c_i, pi_1_p_i);
        
taulist

