function joints = invKin(X, Y, Z, TX, TY, TZ, HH, HL)
    tic
%     T3 = pi - asin(Z/12 + (HH*cos(TY))/12 - (HL*sin(TY))/12 - 7/4)
    T3 = asin(Z/12 + (HH*cos(TY))/12 - (HL*sin(TY))/12 - 7/4)

%     TK = TY - pi + asin(Z/12 + (HH*cos(TY))/12 - (HL*sin(TY))/12 - 7/4)
    TK = TY - asin(Z/12 + (HH*cos(TY))/12 - (HL*sin(TY))/12 - 7/4)

    DT = 12*cos(T3) + HL*cos(T3+TK) + HH*sin(T3+TK)

    T0 = 2*atan((16*(Y + Y*tan(TZ/2)^2 - 2*DT*tan(TZ/2)))/(8*X - 8*DT + 8*DT*tan(TZ/2)^2 + 8*X*tan(TZ/2)^2 + DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2) - (8*Y + 8*Y*tan(TZ/2)^2 - (-(DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2)*(DT^2*tan(TZ/2)^2 - 64*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2 - 64))^(1/2) - 16*DT*tan(TZ/2))/(8*X - 8*DT + 8*DT*tan(TZ/2)^2 + 8*X*tan(TZ/2)^2 + DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2))
    % T0 = 2*atan((16*(Y + Y*tan(TZ/2)^2 - 2*DT*tan(TZ/2)))/(8*X - 8*DT + 8*DT*tan(TZ/2)^2 + 8*X*tan(TZ/2)^2 + DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2) - (8*Y + 8*Y*tan(TZ/2)^2 + (-(DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2)*(DT^2*tan(TZ/2)^2 - 64*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2 - 64))^(1/2) - 16*DT*tan(TZ/2))/(8*X - 8*DT + 8*DT*tan(TZ/2)^2 + 8*X*tan(TZ/2)^2 + DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2))

    T2 = TZ - 2*atan((8*Y + 8*Y*tan(TZ/2)^2 - (-(DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2)*(DT^2*tan(TZ/2)^2 - 64*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2 - 64))^(1/2) - 16*DT*tan(TZ/2))/(8*X - 8*DT + 8*DT*tan(TZ/2)^2 + 8*X*tan(TZ/2)^2 + DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2))
    % T2 = TZ - 2*atan((8*Y + 8*Y*tan(TZ/2)^2 + (-(DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2)*(DT^2*tan(TZ/2)^2 - 64*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2 - 64))^(1/2) - 16*DT*tan(TZ/2))/(8*X - 8*DT + 8*DT*tan(TZ/2)^2 + 8*X*tan(TZ/2)^2 + DT^2*tan(TZ/2)^2 + X^2*tan(TZ/2)^2 + Y^2*tan(TZ/2)^2 + DT^2 + X^2 + Y^2 - 2*DT*X - 4*DT*Y*tan(TZ/2) + 2*DT*X*tan(TZ/2)^2))

    T1 = TZ - T0 - T2

    k = sqrt((12*cos(T3)+2*cos(T3+TK-pi/2))^2 + (12*cos(T3)+2*sin(T3+TK-pi/2)-2)^2) 
    f = sqrt(12^2 + 2^2 - 2*2*12*cos(pi/2 + TK))

    T4 = pi - acos((2^2 + k^2 - f^2)/(2*2*k)) - acos((k^2 + 4^2 - 13^2)/(2*4*k))
    
    joints = [T0 T1 T2 T3 T4 TK] 
    toc
end



