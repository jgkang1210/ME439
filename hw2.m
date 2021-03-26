% 20190348 강준길
addpath('C:\ws\git\ModernRobotics\packages\MATLAB\mr')

% pg.30
disp("---------");
disp("pg30");
omega = [
        0;
        0.866;
        0.5];

theta = pi/6;

omega_theta = omega * theta;

so3_omega_theta = VecToso3(omega_theta);

R = MatrixExp3(so3_omega_theta);

logR = MatrixLog3(R);

soln_omega_theta = so3ToVec(logR);

[soln_omega, soln_theta] = AxisAng3(soln_omega_theta);

omega, theta, R, soln_omega, soln_theta

% pg.47
disp("---------");
disp("pg47");

% lengths of arms
L1 = 1;
L2 = 2;

Twist = [0 0 1 0 -(L1 + L2) 0];

omega = Twist(1:3);
velocity = Twist(4:6);

q = [(L1 + L2) 0 0];

v = - cross(omega, q);

v, velocity

% pg.48
disp("---------");
disp("pg48");

% lengths of arms
L1 = 1;
L2 = 2;
L3 = 3;

% zero position
M = [
    1 0 0 L1 + L2 + L3;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1;];

V1s = [0 0 1 0 0 0]';
V2s = [0 0 1 0 -L1 0]';
V3s = [0 0 1 0 -(L1 + L2) 0]';
V1b = [0 0 1 0 (L1 + L2 + L3) 0]';
V2b = [0 0 1 0 (L2 + L3) 0]';
V3b = [0 0 1 0 L3 0]';

Vs = [V1s V2s V3s];
Vb = [V1b V2b V3b];

Adm = Adjoint(M);

Vtemp = Adm * Vb;

Vs, Vtemp

% Vs = Adjoint(M) * Vb 라는 결과가 만족된다.

% pg.49
disp("---------");
disp("pg49");

L1 = 1;
L2 = 2;

% zero position
M = [
    0 0 1 L1;
    0 1 0 0;
    -1 0 0 -L2;
    0 0 0 1;];

V1s = [0 0 1 0 0 0]';
V2s = [0 -1 0 0 0 -L1]';
V3s = [1 0 0 0 -L2 0]';
V1b = [-1 0 0 0 L1 0]';
V2b = [0 -1 0 0 0 L2]';
V3b = [0 0 1 0 0 0]';

Vs = [V1s V2s V3s];
Vb = [V1b V2b V3b];

Adm = Adjoint(M);

Vtemp = Adm * Vb;

Vs, Vtemp

% Vs = Adjoint(M) * Vb 라는 결과가 만족된다.

% pg.50
disp("---------");
disp("pg50");

% parameters
qs = [2 -1 0]';
qb = [2 -1.4 0]';
ws = [0 0 2]';
wb = [0 0 -2]';

% pitch
h = 0;

% calulate twist from information about screw motion
Vs = ScrewToAxis(qs, ws, h);
Vb = ScrewToAxis(qb, wb, h);

Tsb = [
    -1 0 0 4;
    0 1 0 0.4;
    0 0 -1 0;
    0 0 0 1;];

AdTsb = Adjoint(Tsb);

checkVs = AdTsb * Vb;

[Sv, thetav] = AxisAng6(Vs);
[Sb, thetab] = AxisAng6(Vb);

Vb, Vs, checkVs, Sv, thetav, Sb, thetab