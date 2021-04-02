% 20190348 Jungill Kang

% return the SE3 matrix for given theta, d, a, alpha of DH table

function T = DH(theta, d, a, alpha)
    % D-H parameter - Rot(z, theta)
    T11 = [
        cos(theta) -sin(theta) 0 0;
        sin(theta) cos(theta) 0 0;
        0 0 1 0;
        0 0 0 1;];
    % D-H parameter - Trans(z, d)
    T12 = [
        1 0 0 0;
        0 1 0 0;
        0 0 1 d;
        0 0 0 1];
    % D-H parameter - Trans(x, a)
    T13 = [
        1 0 0 a;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];
    % D-H parameter - Rot(x, alpha)
    T14 = [
        1 0 0 0;
        0 cos(alpha) -sin(alpha) 0;
        0 sin(alpha) cos(alpha) 0;
        0 0 0 1];
    % return the SE3 matrix
    T = T11*T12*T13*T14;

