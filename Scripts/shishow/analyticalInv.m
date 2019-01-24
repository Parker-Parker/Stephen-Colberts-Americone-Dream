function q = analyticalInv(vect,HH,HL, varargin)
%     q = [TH0 TH1 TH2 TH3 TH4] vect = [x,y,z,tx,ty,tz]
    mode = "right";
    for n = varargin(:)
        if n == "right"
            mode = "right";
        elseif  n == "left"
            mode = "left";
        elseif length(n) == 5
            mode = "fromJoint";
        elseif length(n) == 6
            mode = "fromWorld";
        end
    end
    
    X = vect(1);
    Y = vect(2);
    Z = vect(3);
    THX = vect(4);
    THY = vect(5);
    THZ = vect(6);
    
    TH3 = asin((Z-21-HL*sin(-THY)+HH*cos(THY))/12);
    THK = -THY -TH3;
    
        F = sqrt(12^2+2^2-2*2*12*cos(pi/2+THK));
            P1 = 12*[cos(TH3) sin(TH3)];
            P2 = P1+2*[cos(TH3+THK-pi/2) sin(TH3+THK-pi/2)];
        K = sqrt(P2(1)^2 + (P2(2)-2)^2); 

    TH4 = pi - acos((2^2+K^2-F^2)/(2*2*K)) - acos((4^2+K^2-13^2)/(2*4*K));
    
    dt = 12*cos(TH3) + HH*sin(TH3+THK) + HL*cos(TH3+THK);
    D2 = [X-dt*cos(THZ), Y-dt*sin(THZ)];
    R = sqrt(D2(1)^2 + D2(2)^2);
    
    AL = acos((4^2+4^2-R^2)/(2*4*4));
    BE = atan2(D2(2),D2(1));
    GA = (pi-AL)/2;
    
    TH0 = BE-GA;
    TH1 = pi - AL;
    TH2 = THZ - TH0 - TH1;
    
    
    
    q = [TH0 TH1 TH2 TH3 TH4 THK];
end