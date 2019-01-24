for n = 1:30

    t = num2cell([0 0 0 0 n*pi/180 1 2]);
    [TH0, TH1, TH2, TH3, TH4, HH, HL] = deal(t{:});

    run1 = ForwardExplicit(TH0, TH1, TH2, TH3, TH4, HH, HL);
    inv1 = analyticalInv(run1,HH,HL);

    t = num2cell([inv1(1:5) HH HL]);
    [TH0, TH1, TH2, TH3, TH4, HH, HL] = deal(t{:});

    run2 = ForwardExplicit(TH0, TH1, TH2, TH3, TH4, HH, HL);
%     run1 - run2;
%     n*pi/180-inv(5)

end