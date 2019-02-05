
syms P2X P2Y TH0 TH1

F2 = P2X == 4*cos(TH0) + 4*cos(TH0+TH1);
F1 = P2Y == 4*sin(TH0) + 4*sin(TH0+TH1);

X = solve(F1,F2,TH0,TH1)
X.TH0 = simplify(X.TH0)
X.TH1 = simplify(X.TH1)