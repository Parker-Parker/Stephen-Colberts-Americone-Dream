function [mats, pts] = fwdMats(TH0, TH1, TH2, TH3, TH4, HH, HL, varargin)
    THK = -pi/2 + acos((sin(TH3) + 2*sin(TH3 - TH4) + 6)/(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)) + acos((16*cos(TH4) + 48*sin(TH3) + 96*sin(TH3 - TH4) - 1)/(8*(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)));
%     THK = -pi/2 + acos((2*sin(TH3) + 4*sin(TH3 - TH4) + 11)/(16*cos(TH4) + 44*sin(TH3) + 88*sin(TH3 - TH4) + 141)^(1/2)) + acos((16*cos(TH4) + 44*sin(TH3) + 88*sin(TH3 - TH4) + 1)/(4*(16*cos(TH4) + 44*sin(TH3) + 88*sin(TH3 - TH4) + 141)^(1/2)))
%     THK = -pi/2 + acos((2*sin(TH3) + 4*sin(TH3 - TH4) + 11)/(16*cos(TH4) + 44*sin(TH3) + 88*sin(TH3 - TH4) + 141)^(1/2)) + acos((16*cos(TH4) + 44*sin(TH3) + 88*sin(TH3 - TH4) + 1)/(4*(16*cos(TH4) + 44*sin(TH3) + 88*sin(TH3 - TH4) + 141)^(1/2)))
    PI = pi;
    params1 = [ 4,     0,  2,      TH0; ...
                4,     0,  2,      TH1; ...
                0,  PI/2, 17,      TH2; ...
               12,     0,  0,      TH3; ...
               HH,     0,  0, THK-PI/2; ...
               HL, -PI/2,  0, 	  PI/2];    %STATIC FRAME OF END EFFECTOR
    X = FwdKinDH(params1);

    T0toT = cell(size(X,2),1);
    for c = 1:size(X,2)
        T0toT(c) = {eye(4)};
        for n = 1:c
            T0toT(c) = {T0toT{c}*X{n}};
        end
    end
    listofpoints = zeros([1+size(T0toT,1), 7]);
    for k = 1:size(T0toT,1)
        listofpoints(k+1,:) = cat(2, extractPose(T0toT{k}), THK);

    end
    
    pts = listofpoints;
    mats = T0toT;
end