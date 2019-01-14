clear; clc; close all;
tic
syms L0 L1 L2 L3 L4 TH4 TH3 THK
PI = sym(1)*pi;

P0 = [0 0];
P4 = [0 -L0];
P1 = P0 + [L4*cos(TH3)      L4*sin(TH3)];
P3 = P4 + [L1*cos(TH4-PI/2) L1*sin(TH4-PI/2)];

H = sqrt((P1(1)-P3(1))^2+(P1(2)-P3(2))^2);
Q = sqrt(L0^2+L1^2 - 2*L0*L1*cos(PI-TH4));
AL = acos((L4^2+H^2-Q^2)/(2*H*L4));
BE = acos((L3^2+H^2-L2^2)/(2*H*L3));

P2 = P1 + [L3*cos(TH3+AL+BE-PI) L3*sin(TH3+AL+BE-PI)];

Simple = simplify(P2);
Theta2 = AL+BE+PI;
Theta2 = simplify(Theta2)

eqn = THK == Theta2;
%solve(eqn, TH4)

pltooo = cat(1, [0 -12], P0, P1, P2, P3, P4);

t = num2cell([2 4 13 2 12]);
[L0, L1, L2, L3, L4] = deal(t{:});

Theta2 = simplify(subs(Theta2))
% pltooo = double(subs(pltooo))

% Q = double(subs(Q))
% H = double(subs(H))
% AL = double(subs(AL))
% BE = double(subs(BE))


% fig = figure();
% plot(pltooo(:,1), pltooo(:,2));
% xlim([-5 15]);
% ylim([-15 10]);
% axis equal;
toc