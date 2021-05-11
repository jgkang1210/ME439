% 20190348 Jungill Kang
addpath('..\mr')

clear;

%% Robot's Properties from urdf file

% length (mm)
L1 = 0.089159;
L2 = 0.13585;
L3 = 0.425;
L4 = 0.1197;
L5 = 0.39225;
L6 = 0.093;
L7 = 0.09465;
L8 = 0.0823;

% mass (kg)
m = [3.7; 8.393; 2.275; 1.219; 1.219; 0.1897];

% Moment of inertia (kg*m^2)
I1 = diag([0.010267495893,0.010267495893,0.00666]);
I2 = diag([0.22689067591,0.22689067591,0.0151074]);
I3 = diag([0.049443313556,0.049443313556,0.004095]);
I4 = diag([0.111172755531,0.111172755531,0.21942]);
I5 = diag([0.111172755531,0.111172755531,0.21942]);
I6 = diag([0.0171364731454,0.0171364731454,0.033822]);

% Ilist = cat(3, I1, I2, I3, I4, I5, I6);

% G list, spatial inertia matrix
G1 = diag([I1(1,1), I1(2,2), I1(3,3), m(1), m(1), m(1)]);
G2 = diag([I2(1,1), I2(2,2), I2(3,3), m(2), m(2), m(2)]);
G3 = diag([I3(1,1), I3(2,2), I3(3,3), m(3), m(3), m(3)]);
G4 = diag([I4(1,1), I4(2,2), I4(3,3), m(4), m(4), m(4)]);
G5 = diag([I5(1,1), I5(2,2), I5(3,3), m(5), m(5), m(5)]);
G6 = diag([I6(1,1), I6(2,2), I6(3,3), m(6), m(6), m(6)]);
Glist = cat(3, G1, G2, G3, G4, G5, G6);

M01 = [
        1 0 0 0;
        0 1 0 0;
        0 0 1 L1;
        0 0 0 1];
M12 = [
        0 0 1 0;
        0 1 0 L2;
        -1 0 0 0;
        0 0 0 1];
M23 = [
        1 0 0 0;
        0 1 0 -L4;
        0 0 1 L3;
        0 0 0 1];
M34 = [
        0 0 1 0;
        0 1 0 0;
        -1 0 0 L5;
        0 0 0 1];
M45 = [
        1 0 0 0;
        0 1 0 L6;
        0 0 1 0;
        0 0 0 1];
M56 = [
        1 0 0 0;
        0 1 0 0;
        0 0 1 L7;
        0 0 0 1];
M67 = [
        1 0 0 0;
        0 0 1 L8;
        0 -1 0 0;
        0 0 0 1];
    
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

Slist = [[0; 0; 1; 0; 0; 0], ...
       [0; 1; 0; -L1; 0; 0], ...
       [0; 1; 0; -L1; 0; L3], ...
       [0; 1; 0; -L1; 0; L3+L5], ...
       [0; 0; 1; -L2+L4-L6; L3+L5; 0], ...
       [0; 1; 0; L7-L1; 0; L3+L5]];

% initial condition
thetalist = [0; 0; 0; 0; 0; 0];
dthetalist = [0; 0; 0; 0; 0; 0];
taulist = [0; 0; 0; 0; 0; 0];
ddthetalist = [0; 0; 0; 0; 0; 0];
Ftip = [0; 0; 0; 0; 0; 0];

% gravity
g = [0; 0; -9.8];

time = 0;
timestep = 0.01;

while time < 3
    % simulated theta
    disp(rad2deg(thetalist'));
    ddthetalist = ForwardDynamics(thetalist, dthetalist, taulist, g, ...
                    Ftip, Mlist, Glist, Slist);
    thetalist = thetalist + dthetalist * timestep;
    dthetalist = dthetalist + ddthetalist * timestep;
    time = time + timestep;
end