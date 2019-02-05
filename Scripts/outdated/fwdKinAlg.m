function ee = fwdKinAlg(TH0, TH1, TH2, TH3, TH4, HH, HL)
%     THK = acos((sin(TH3)-2*sin(TH4-TH3)+6) ...
%                     /sqrt(4*cos(TH4)+12*sin(TH3)-24*sin(TH4-TH3)+41)) ...
%                     +acos((16*cos(TH4)+48*sin(TH3)-96*sin(TH4-TH3)-1) ...
%                       /(8*sqrt(4*cos(TH4)+12*sin(TH3)-24*sin(TH4-TH3)+41)))-pi/2;
    THK = -pi/2 + acos((sin(TH3) + 2*sin(TH3 - TH4) + 6)/(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)) + acos((16*cos(TH4) + 48*sin(TH3) + 96*sin(TH3 - TH4) - 1)/(8*(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)));

    TZ = TH0+TH1+TH2;
    TX = 0;
    TY = TH3+THK;
    
    DT = 12*cos(TH3)+HL*cos(TH3+THK)+HH*sin(TH3+THK);
    
    X = 4*cos(TH0) + 4*cos(TH0+TH1) + DT*cos(TH0+TH1+TH2);
    Y = 4*sin(TH0) + 4*sin(TH0+TH1) + DT*sin(TH0+TH1+TH2);
    Z = 2+2+17+12*sin(TH3)+HL*sin(TH3+THK)+HH*cos(TH3+THK);
    ee = [X,Y,Z,TX,TY,TZ,THK];
end