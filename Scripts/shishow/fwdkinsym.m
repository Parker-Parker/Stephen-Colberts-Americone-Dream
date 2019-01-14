clc; clear; close all;
syms T0 T1 T2 T3 T4 HH HL
ee = dofwdfast(T0, T1, T2, T3, T4, HH, HL);
disp(ee')
disp(simplify(ee'))
HH = 1
HL = 2
disp(simplify(subs(ee')))