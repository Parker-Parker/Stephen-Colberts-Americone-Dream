% kinematics test
clear; clc; close all;

% TEST INVERSE SKEET

[N(1),N(2),N(3),N(4),N(5),N(6)] = Forwardplswork(0, 0, 0, -pi/18, 0*pi/18, 1, 2);


[J1, J2, J3, J4, J5] = Inverseplswork(N(1),N(2),N(3),N(4),N(5),N(6),1,2);

[N2(1),N2(2),N2(3),N2(4),N2(5),N2(6)] = Forwardplswork(J1,J2,J3,J4,J5, 1, 2);

F = Forwardpts(0, 0, 0, -pi/18, 0*pi/18, 1, 2)

plot3(F(:,1),F(:,2),F(:,3),F(:,4),F(:,5),F(:,6));
axis equal;

N2

% [J1, J2, J3, J4, J5];
