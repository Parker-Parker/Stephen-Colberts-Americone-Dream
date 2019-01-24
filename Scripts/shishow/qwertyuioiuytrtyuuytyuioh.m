clc; clear; close all;
syms TH0 TH1 TH2 TH3 TH4 HH HL

ee = dofwdfast(TH0, TH1, TH2, TH3, TH4, HH, HL)

jack = jacobian(ee, [TH0 TH1 TH2 TH3 TH4])


