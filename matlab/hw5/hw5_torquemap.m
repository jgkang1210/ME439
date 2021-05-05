% 20190348 Jungill Kang
addpath('..\mr')

clear;

%% open chain robot

theta1 = pi/4;
theta2 = pi/4;

s1 = sin(theta1);
s2 = sin(theta2);
c1 = cos(theta1);
c2 = cos(theta2);

M = [
    8 + 2*s2^2 2*c2;
    2*c2 6;];

figure(1);
title('Torque map');
xlabel('t1');
ylabel('t2');

% make unit circle
% scale factor for circle
scale = 1;
for alpha = 0 : 0.01 : 2*pi
    cosAlpha = scale * cos(alpha);
    sinAlpha = scale * sin(alpha);
    thetaDdot = [cosAlpha sinAlpha]';
    
    % mapping each value
    torque = M * thetaDdot;
    
    % draw the point on the graph
    plot(thetaDdot(1), thetaDdot(2), 'ro', 'MarkerSize', 1);
    plot(torque(1), torque(2), 'go', 'MarkerSize', 1);
    hold on;
end

axis square; % square graph

grid ON;

xlim([-12, 12]); % x range
ylim([-12, 12]); % y range