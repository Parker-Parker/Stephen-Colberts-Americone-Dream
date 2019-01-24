function ee = ForwardExplicit(TH0, TH1, TH2, TH3, TH4, HH, HL, varargin)
    
    THK = -pi/2 + acos((sin(TH3) + 2*sin(TH3 - TH4) + 6)/(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)) + acos((16*cos(TH4) + 48*sin(TH3) + 96*sin(TH3 - TH4) - 1)/(8*(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)));
    dt = 12*cos(TH3) + 1*sin(TH3+THK) + 2*cos(TH3+THK);

    HH = 1;
    HL = 2;

    points = zeros(7,7);
%     points(1,:) = [0 1 0 0 0 0 0];
    points(2,:) = [4*cos(TH0), 4*sin(TH0), 2, 0, 0, TH0, THK] + points(1,:);
    points(3,:) = [4*cos(TH0+TH1), 4*sin(TH0+TH1), 2, 0, 0, TH1, 0] + points(2,:);
    points(4,:) = [0, 0, 17, 0, 0, TH2, 0] + points(3,:);
    points(5,:) = [12*cos(TH3)*cos(TH0+TH1+TH2), 12*cos(TH3)*sin(TH0+TH1+TH2), 12*sin(TH3), 0, -TH3, 0, 0] + points(4,:);
    points(6,:) = [HH*sin(TH3+THK)*cos(TH0+TH1+TH2), HH*sin(TH3+THK)*sin(TH0+TH1+TH2), -HH*cos(TH3+THK), 0, -THK, 0, 0] + points(5,:);
    points(7,:) = [HL*cos(TH3+THK)*cos(TH0+TH1+TH2), HL*cos(TH3+THK)*sin(TH0+TH1+TH2), HL*sin(TH3+THK), 0, 0, 0, 0] + points(6,:);

    if nargin>7 && varargin(1)=="full" 
        ee = points(:,1:6);
    else
        ee = points(7,1:7);
    end

end