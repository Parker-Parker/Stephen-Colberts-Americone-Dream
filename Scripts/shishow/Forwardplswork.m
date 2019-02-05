function [X,Y,Z,THX,THY,THZ] = Forwardplswork(TH0, TH1, TH2, TH3, TH4,HH,HL)
    
    TL = 13; BL = 14; GL = 2; FL = 2; LL = 4;
    t = 21;
    
        PPY = -GL - LL*cos(TH4); 
        PPX = LL*sin(TH4); 
         
        PQY = TL*sin(TH3); 
        PQX = TL*cos(TH3);        
        
        P = sqrt(PPY^2+PPX^2);
        Q = sqrt((PPY-PQY)^2+(PPX-PQX)^2);
        
        ALPHA = acos((Q^2+TL^2-P^2)/(2*Q*TL));
        BETA  = acos((Q^2+FL^2-BL^2)/(2*Q*FL));
   
        THK = ALPHA + BETA - pi/2;
    
    % -THY = TH3+THK
    % Z = t + 12*sin(TH3) - HH*cos(TH3+THK) + HL*sin(TH3+THK)
    
    Z = t + TL*sin(TH3) - HH*cos(TH3+THK) + HL*sin(TH3+THK);
    THY = -(TH3+THK);

    dt =  TL*cos(TH3) + HH*sin(-THY) + HL*cos(-THY);

    THZ = TH0 + TH1 + TH2;
    
    X = 4*cos(TH0) + 4*cos(TH0+TH1) + dt*cos(THZ);
    Y = 4*sin(TH0) + 4*sin(TH0+TH1) + dt*sin(THZ);

    THX = 0;
end
