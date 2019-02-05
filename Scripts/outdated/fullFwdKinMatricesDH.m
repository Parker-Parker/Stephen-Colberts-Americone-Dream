clear; clc; close all;
syms TH0 TH1 TH2 TH3 THK HH HL
PI = sym(1)*pi;
params = [ 4,     0,  2,      TH0; ...
           4,     0,  2,      TH1; ...
           0,  PI/2, 17,      TH2; ...
          13,     0,  0,      TH3; ...
          HH,     0,  0, THK-PI/2; ...
          HL, -PI/2,  0, 	 PI/2]    %STATIC FRAME OF END EFFECTOR
X = FwdKinDH(params)
celldisp(X)
X = cellfun(@simplify,FwdKinDH(params),'un',0)
celldisp(X)





