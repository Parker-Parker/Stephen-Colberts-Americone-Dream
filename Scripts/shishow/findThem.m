%%% find singulaarities
TH3 = 0;
TH4 = 0;
HH = 1;
HL = 2;
i = 0
listpts2 = [0 0 0 0 0];
for TH0 = 0*pi/180:3*pi/180:360*pi/180
    
    for TH1 = 0*pi/180:3*pi/180:360*pi/180
    
        for TH2 = 0*pi/180:3*pi/180:0*pi/180
            i = i+1
            if det(jacobGetSquare(TH0, TH1, TH2, TH3, TH4, HH, HL)) == 0
                listpts2 = [listpts; TH0 TH1 TH2 TH3 TH4]
            end
        end
    end
end



                
                