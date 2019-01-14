clc; clear; close all;
tic
syms T0 T1 T2 T3 TK HH HL X Y Z TX TY TZ DT

% DT = 12*cos(T3) + HL*cos(T3+TK) + HH*sin(T3+TK)

eq1 = X == 4*cos(T0) + 4*cos(T0+T1) + DT*cos(T0+T1+T2)
eq2 = Y == 4*sin(T0) + 4*sin(T0+T1) + DT*sin(T0+T1+T2)
eq3 = Z == 2 + 2 + 17 + 12*sin(T3) + HL*sin(T3+TK) - HH*cos(T3+TK)

eq4 = TX == 0
eq5 = TY == T3 + TK
eq6 = TZ == T0 + T1 + T2

constraint = T1 >= 0
constraint2 = T1 <= pi
constraint3 = T0 >= 0
constraint4 = T0 < pi*2
constraint5 = T2 >= 0
constraint6 = T2 < pi*2
kek = solve(eq3, eq5, T3, TK)
kok = solve(eq1, eq2, eq4, eq6, constraint2, constraint, constraint3, constraint4, constraint5, constraint6, T0, T1, T2)

kik = solve([X==4*cos(T0)+4*cos(TZ-T2)+DT*cos(TZ);Y==4*sin(T0)+4*sin(TZ-T2)+DT*sin(TZ)],[T0 T2])
% % % % % CONST1 = T1 >= 0
% % % % % CONST2 = abs(TK) <= pi/2
% % % % eqns = [eq1; eq2; eq3; eq4; eq5; eq6; ...
% % % %             T0 >= 0; T0 < 2*pi; ...
% % % %             T1 >= 0; T1 < pi; ...
% % % %             T2 >= 0; T2 < pi; ...
% % % %             TK >= -pi/2; TK <= pi/2] 
% % % % outs = [T0 T1 T2 T3 TK]      
% % % % toc
% % % % % xxyyzz = solve(eq1, eq2, eq3, eq4, eq5, eq6, CONST1, CONST2, T0, T1, T2, T3, TK, 'IgnoreAnalyticConstraints',true, 'ReturnConditions', true)
% % % % [TT0, TT1, TT2, TT3, TTK] = solve(eqns, outs, 'IgnoreAnalyticConstraints',true) %, 'ReturnConditions', true)
% % % % toc