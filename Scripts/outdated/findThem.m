%%% find singulaarities
TH3 = 0;
TH4 = 0;
HH = 1;
HL = 2;
i = 0
listpts2 = zeros(180^3,5);
for TH0 = 0*pi/180:2*pi/180:360*pi/180
    
    for TH1 = 0*pi/180:2*pi/180:360*pi/180
    
        for TH2 = 0*pi/180:2*pi/180:360*pi/180
            
            for TH3 = 0*pi/180:2*pi/180:0*pi/180
                
                for TH4 = 0*pi/180:2*pi/180:0*pi/180
                    
                    try
                        tic
                        if det(jacobGetSquare(TH0, TH1, TH2, TH3, TH4, HH, HL)) == 0
                            listpts2 = [listpts2; TH0 TH1 TH2 TH3 TH4];
                        end
                        i = i+1
                        toc
                    end
                end    
            end
        end
    end
end



                
                