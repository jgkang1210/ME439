% 20190348 Jungill Kang
addpath('..\mr')

clear;
clc;

%% Properties
L1 = 2;
L2 = 2;
L3 = 1;

T0 = [
    0 -1 0 2;
    1 0 0 3;
    0 0 1 0;
    0 0 0 1];

Td = [
    1 0 0 1;
    0 1 0 -2;
    0 0 1 0;
    0 0 0 1];

Xe = se3ToVec(MatrixLog6(T0\Td));

% desired Trajectory
Xd = Td;

% step size
dt = 0.01;
n = 1/dt;

% desired twist, 0
Vd = zeros(6, 1);

%% calculate the trajectory

X = T0;

Kp = 10;

Blist = [
    [0;0;1;L1;L2+L3;0], ...
    [0;0;1;0;L2+L3;0], ...
    [0;0;1;0;L3;0]];

thetalist = [0; 0; 0];

M = [
    0 -1 0 2;
    1 0 0 3;
    0 0 1 0;
    0 0 0 1;];

figure(1);
title('point of robot');
xlabel('x');
ylabel('y');

axis square; % square graph

xlim([-4, 4]); % x range
ylim([-4, 4]); % y range

for i = 1:1:n
    Vb = Kp * se3ToVec(MatrixLog6(X\Xd));
    
    J = JacobianBody(Blist, thetalist);
    
    Jinv = (J'*J)\J';
    
    thetadotlist = Jinv * Vb;
    
    joint1 = [L1 * cos(thetalist(1)); L1 * sin(thetalist(1)); 0];
    joint2 = [L1 * cos(thetalist(1)) + L2 * cos(thetalist(1)+thetalist(2)+pi/2); L1 * sin(thetalist(1)) + L2 * sin(thetalist(1)+thetalist(2)+pi/2); 0];
    
    % draw the point on the graph
    hold on;
    
    originX = [[0,0], joint1(1), joint2(1), X(1,4)];
    originY = [[0,0], joint1(2), joint2(2), X(2,4)];
    
    plot(originX, originY, 'o:', 'linewidth', 2);
    
    thetalist = thetalist + thetadotlist * dt;
    
    X = FKinBody(M, Blist, thetalist); 
end