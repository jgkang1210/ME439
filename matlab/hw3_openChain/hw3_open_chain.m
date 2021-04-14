% 20190348 Jungill Kang
addpath('..\mr')

clear;

%% open chain robot

% get the length of open chain robot
L1 = 1;
L2 = 1;
%L1 = input("What is L1 of robot : ");
%L2 = input("What is L2 of robot : ");

% make the theta list for the configuration
theta1 = 180 * (pi/180);
theta2 = 20 * (pi/180);
thetalist = [theta1 theta2]';

% initial(theta 1 = 0, theta 2 = 0) SE3 matrix of L1 
M1 = [1 0 0 L1;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1;];

% initial(theta 1 = 0, theta 2 = 0) SE3 matrix of L2
M2 = [1 0 0 L1+L2;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1;];

% space screw axis at theta 1 = 0, theta 2 = 0
S1 = [0 0 1 0 0 0]';
S2 = [0 0 1 0 -L1 0]';
Slist = [S1 S2];

% body screw axis at theta 1 = 0, theta 2 = 0
B1 = [0 0 1 0 L1+L2 0]';
B2 = [0 0 1 0 L2 0]';
Blist = [B1 B2];

% Forward kinematics
T1 = FKinSpace(M1, S1, theta1);
T2 = FKinSpace(M2, Slist, thetalist);

% get the two starting and end point from each SE3 matrix
[R1, p1] = TransToRp(T1);
[R2, p2] = TransToRp(T2);

% plot the robot arm

% plot L1
x1 = [0 p1(1)];
y1 = [0 p1(2)];

% plot L2
x2 = [p1(1) p2(1)];
y2 = [p1(2) p2(2)];

figure(1);
title('2R open chain');
xlabel('x');
ylabel('y');

grid ON;
axis square; % square graph

xlim([-2.5, 2.5]); % x range
ylim([-2.5, 2.5]); % y range

line(x1, y1, 'Color', 'red');
text(p1(1)/2, p1(2)/2 - 0.1, 'L1');
hold on;
line(x2, y2, 'Color', 'blue');
text((p1(1)+p2(1))/2 + 0.1, (p1(2)+p2(2))/2, 'L2');
hold on;

% now draw the manipulability ellipsoid

% Since we need analytic jacobian for manipulability
J_analytic = [
    -L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2);
    L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2);];

% make unit circle
% scale factor for circle
scale = 0.2;
for alpha = 0 : 0.01 : 2*pi
    cosAlpha = scale * cos(alpha);
    sinAlpha = scale * sin(alpha);
    thetaDot = [cosAlpha sinAlpha]';
    
    % mapping each value
    xDot = J_analytic * thetaDot;
    
    % draw the point on the graph
    plot(xDot(1) + p2(1), xDot(2) + p2(2), 'go', 'MarkerSize', 1);
    hold on;
end

% Now calculate three measure

% decompose the eigenvalue from J_analytic * J_analytic transpose
Evalue = eig(J_analytic * J_analytic');

minEvalue = min(Evalue);
maxEvalue = max(Evalue);

firstMeasure = sqrt(maxEvalue / minEvalue);
firstMeasureText = "first measure : " + firstMeasure;
text(-0.9, 1.8, firstMeasureText);

secondMeasure = maxEvalue / minEvalue;
secondMeasureText = "second measure : " + secondMeasure;
text(-0.9, 1.7, secondMeasureText);

thirdMeasure = sqrt(det(J_analytic * J_analytic'));
thirdMeasureText = "third measure : " + thirdMeasure;
text(-0.9, 1.6, thirdMeasureText);
