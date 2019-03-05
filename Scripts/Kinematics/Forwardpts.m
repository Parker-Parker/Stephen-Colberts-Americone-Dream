function pts = Forwardpts(TH0, TH1, TH2, TH3, TH4,HH,HL)
    
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
    
    pts = [0 0 0]
    pts(2,:) = pts(1,:)+ [4*cos(TH0),                       4*sin(TH0),                         0];
    pts(3,:) = pts(2,:)+ [4*cos(TH0+TH1),                   4*sin(TH0+TH1),                     0];
    pts(4,:) = pts(3,:)+ [0,                                0,                                  21];
    
    pts(5,:) = pts(4,:)+ [TL*cos(TH0+TH1+TH2)*cos(TH3),     TL*sin(TH0+TH1+TH2)*cos(TH3),       TL*sin(TH3)];
    pts(6,:) = pts(5,:)+ [HH*cos(TH0+TH1+TH2)*sin(TH3+THK), HH*sin(TH0+TH1+TH2)*sin(TH3+THK),   -HH*cos(TH3+THK)];
    pts(7,:) = pts(6,:)+ [HL*cos(TH0+TH1+TH2)*cos(TH3+THK), HL*sin(TH0+TH1+TH2)*cos(TH3+THK),   HL*sin(TH3+THK)];

    pts2 = pts;
    pts2(6,:) = pts(5,:)+ [FL*cos(TH0+TH1+TH2)*sin(TH3+THK), FL*sin(TH0+TH1+TH2)*sin(TH3+THK),   -FL*cos(TH3+THK)];
    pts2(8,:) = pts(4,:)+ [FL*cos(TH0+TH1+TH2)*sin(TH3+THK), FL*sin(TH0+TH1+TH2)*sin(TH3+THK),   -FL*cos(TH3+THK)];
    
end
