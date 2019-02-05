clc; clear; close all;
tic
syms T0 T1 T2 T3 TK HH HL X Y Z TX TY TZ

DT = 12*cos(T3) + HL*cos(T3+TK) + HH*sin(T3+TK)

eq1 = X == 4*cos(T0) + 4*cos(T0+T1) + DT*cos(T0+T1+T2)
eq2 = Y == 4*sin(T0) + 4*sin(T0+T1) + DT*sin(T0+T1+T2)
eq3 = Z == 2 + 2 + 17 + 12*sin(T3) + HL*sin(T3+TK) - HH*cos(T3+TK)

eq4 = TX == 0
eq5 = TY == T3 + TK
eq6 = TZ == T0 + T1 + T2

CONST1 = T1 >= 0
CONST2 = abs(TK) <= pi/2

toc
% xxyyzz = solve(eq1, eq2, eq3, eq4, eq5, eq6, CONST1, CONST2, T0, T1, T2, T3, TK, 'IgnoreAnalyticConstraints',true, 'ReturnConditions', true)
[TT0, TT1, TT2, TT3, TTK, param, cond] = solve(eq1, eq2, eq3, eq4, eq5, eq6, CONST1, CONST2, T0, T1, T2, T3, TK, 'IgnoreAnalyticConstraints',true, 'ReturnConditions', true)
toc

% % % assume(cond)
% % % interval = [solx > 0, solx < 2*pi];
% % % solk = solve(interval, param)
% % % valx = subs(solx, param, solk)
% % assume(cond)
% % interval = [TT0 > 0, TT0 < 2*pi, ...
% %             TT1 > 0, TT1 < 2*pi, ...
% %             TT2 > 0, TT2 < 2*pi, ...
% %             TT3 > 0, TT3 < 2*pi, ...
% %             TTK > 0, TTK < 2*pi];
% % solk = solve(interval, param)
% % [vTT0, vTT1, vTT2, vTT3, vTTK] = subs([TT0, TT1, TT2, TT3, TTK], param, solk)


% yeet = {xxyyzz.T0, xxyyzz.T1, xxyyzz.T2, xxyyzz.T3, xxyyzz.TK}
toc
