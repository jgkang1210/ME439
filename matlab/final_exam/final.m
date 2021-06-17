r = 0.5;
d = 1;
L1 = 3;
xr = 2;

theta = pi/3;
phi = pi*2;

phi

M1= [
    1 0 0;
    xr*sin(theta) cos(theta+phi) sin(theta+phi);
    xr*cos(theta)+L1 -sin(theta+phi) cos(theta+phi)];

M2 = [
    -r/2*d r/2*d;
    r/2*cos(phi) r/2*cos(phi);
    r/2*sin(phi) r/2*sin(phi)];

M1*M2

N1 = [
    1 0 0;
    xr*sin(theta) cos(theta) sin(theta);
    xr*cos(theta)+L1 -sin(theta) cos(theta)];

N2 = [
    -r/2*d r/2*d;
    r/2 r/2;
    0 0];

N1 * N2