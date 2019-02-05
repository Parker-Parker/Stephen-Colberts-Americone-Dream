function [TH0, TH1, TH2, TH3, TH4] = Inverseplswork(X,Y,Z,THX,THY,THZ,HH,HL)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
TL = 13; BL = 14;
L0 = 4; L1 = 4;
% -THY = TH3+THK
% Z = t + 12*sin(TH3) - HH*cos(TH3+THK) + HL*sin(TH3+THK)
% 
% Z = t + 12*sin(TH3) - HH*cos(-THY) + HL*sin(-THY)
% -12*sin(TH3) = t -Z - HH*cos(-THY) + HL*sin(-THY)

TH3 = asin((21 -Z - HH*cos(-THY) + HL*sin(-THY))/(-TL));
THK = -THY -TH3;

dt =  TL*cos(TH3) + HH*sin(-THY) + HL*cos(-THY);

% X = P2X + dt*cos(THZ)
P2X = X - dt*cos(THZ);
% Y = P2Y + dt*sin(THZ);
P2Y = Y - dt*sin(THZ);

% TH2 =  2345676543456789876567890%???????????????? KHAAAAAAAANNNNNN!!!!!!!!!

% P2X = 4*cos(TH0) + 4*cos(TH0+TH1);
% P2Y = 4*sin(TH0) + 4*sin(TH0+TH1);

% % % % % TH0 = 2*atan2((8*P2Y + (- P2X^4 - 2*P2X^2*P2Y^2 + 64*P2X^2 - P2Y^4 + 64*P2Y^2)^(1/2)),(P2X^2 + 8*P2X + P2Y^2));
% % % % % TH0 = 2*atan2((8*P2Y - (- P2X^4 - 2*P2X^2*P2Y^2 + 64*P2X^2 - P2Y^4 + 64*P2Y^2)^(1/2)),(P2X^2 + 8*P2X + P2Y^2));
% % % % % 
% % % % % TH1 = -2*atan2((-(P2X^2 + P2Y^2)*(P2X^2 + P2Y^2 - 64))^(1/2),(P2X^2 + P2Y^2));
% % % % % TH1 =  2*atan2((-(P2X^2 + P2Y^2)*(P2X^2 + P2Y^2 - 64))^(1/2),(P2X^2 + P2Y^2));


THR = atan2(P2Y,P2X);
R = sqrt(P2X^2+P2Y^2);

GAMMA = acos((L0^2+R^2-L1^2)/(2*L0*R));

% RIGHT
TH0 = THR - GAMMA;
TH1 = 2*GAMMA;

% % % % % % LEFT
% % % % % TH0 = THR + GAMMA;
% % % % % TH1 = -2*GAMMA;


%NOW IS THE TIME
TH2 = THZ - TH0 - TH1;
%TO DIME

% THK:TH4 DUDE its its own thing... like holy shit
%     TH4 = "KEKSAUCE"
%     F = sqrt((2*2*TL)*cos(pi/2+THK)-TL^2-2^2);
    
    PKX = TL*cos(TH3) + 2*sin(TH3+THK);
    PKY = TL*sin(TH3) - 2*cos(TH3+THK);
    
    F = sqrt(PKX^2+(PKY)^2);
    K = sqrt(PKX^2+(PKY+2)^2);
    
    ALPHA = acos((2^2+K^2-F^2)/(2*2*K));
    BETA = acos((K^2+4^2-BL^2)/(K*4*2));
    TH4 = pi - ALPHA - BETA;
%ITS THE TIME OF THE SEASON
outputArg1 = 1;
outputArg2 = 2;

%SOMETHIN SOMETHIN GOOD REASON
end

