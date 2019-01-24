syms TH0 TH1 TH2 TH3 TH4 HH HL
ee = simplify(dofwdfast(TH0, TH1, TH2, TH3, TH4, HH, HL));
pretty(jacobian(ee,[TH0, TH1, TH2, TH3, TH4]))

