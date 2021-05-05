% 20190348 Jungill Kang
addpath('..\mr')

%% prob 4 UR 5 robot

W1 = 0.109;
W2 = 0.082;
L1 = 0.425;
L2 = 0.392;
H1 = 0.089;
H2 = 0.095;

S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -H1 0 0]';
S3 = [0 1 0 -H1 0 L1]';
S4 = [0 1 0 -H1 0 L1+L2]';
S5 = [0 0 -1 -W1 L1+L2, 0]';
S6 = [0 1 0 H2-H1 0 L1+L2]';

Slist = [S1 S2 S3 S4 S5 S6];

M = [
    -1 0 0 L1+L2;
    0 0 1 W1+W2;
    0 1 0 H1-H2;
    0 0 0 1;];

Tsd = [
    0 1 0 0-0.5;
    0 0 -1 0.1;
    -1 0 0 0.1;
    0 0 0 1;];

ew = 0.001;
ev = 0.0001;

thetaini = [0.1 0.1 0.1 0.1 0.1 0.1]';

[thetalist, success] = IKinSpace(Slist, M, Tsd, thetaini, ew, ev)

Tfk = FKinSpace(M, Slist, thetalist)