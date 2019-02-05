clear; clc; close all;
syms L0 L1 L2 L3 L4 TH0 TH1 TH2
PI = sym(1)*pi;

P0 = [0 0];
P4 = [0 -L0];
P1 = P0 + [L4*cos(TH1)      L4*sin(TH1)];
P3 = P4 + [L1*cos(TH0-PI/2) L1*sin(TH0-PI/2)];

H = sqrt((P1(1)-P3(1))^2+(P1(2)-P3(2))^2);
Q = sqrt(L0^2+L1^2 - 2*L0*L1*cos(PI-TH0));
AL = acos((L4^2+H^2-Q^2)/(2*H*L4));
BE = acos((L3^2+H^2-L2^2)/(2*H*L3));

P2 = P1 + [L3*cos(TH1+AL+BE-PI) L3*sin(TH1+AL+BE-PI)];

Simple = simplify(P2);
Theta2 = AL+BE-PI;
Theta2 = simplify(Theta2)

eqn = TH2 == Theta2;
t = num2cell([2 4 13 2 12]);
[L0 L1 L2 L3 L4] = deal(t{:});
THETA2H = simplify(subs(eqn))
solve(simplify(subs(eqn)), TH0)

%pltooo = cat(1, [0 -12], P0, P1, P2, P3, P4)

%t = num2cell([2 4 13 2 12 5*pi/180 0]);
%[L0 L1 L2 L3 L4 TH0 TH1] = deal(t{:});
 
%pltooo = subs(pltooo)
%fig = figure()
%plot(pltooo(:,1), pltooo(:,2))
%xlim([-5 15])
%ylim([-15 10])
%axis equal