clear

% ins = mat2cell([...
%     pi/8, pi/8, pi/8, 0*pi/180, -18*pi/180 ...
%     , 1, 2],[1],[1 1 1 1 1 1 1]);
d = pi/180;
ins = num2cell([...
    
    0 0 0 0 0 1 2 ...
    
    ].*[d d d d d 1 1]);
ee1 = drawfwd(ins{:});%-fwdKinAlg12345(ins{:})
ee1 = ee1(6:-1:4)
ee2 = drawfwd2(ins{:})
ee2 = rotm2eul(ee2(1:3,1:3))
ee3 = ee1-ee2
x = 1:.01:360
ty = zeros(1,36000)
tz = zeros(1,36000)
tx = zeros(1,36000)
for n = 1:.01:360
    try
        ins = num2cell([0 0 0 0 n 1 2].*[d d d d d 1 1]);
%         xxx = (drawfwd(ins{:})-fwdKinAlg(ins{:}))
%         xxx = (dofwdfast12345(ins{:})-fwdKinAlg12345(ins{:}))

            ee1 = drawfwd(ins{:});%-fwdKinAlg12345(ins{:})
            ee1 = ee1(6:-1:4)
            ee2 = drawfwd2(ins{:})
            ee2 = rotm2eul(ee2(1:3,1:3))
            ee3 = ee1-ee2
% 
%         ty(n) = xxx(5);
%         z(n) = xxx(3);
% 
        tz(n*100) = ee3(1);
        ty(n*100) = ee3(2);
        tx(n*100) = ee3(3);
    end
end
plot(x,tx(1:35901),x,ty(1:35901),x,tz(1:35901))
% 
% 
% 
% function ee = dofwdfast12345(TH0, TH1, TH2, TH3, THK, HH, HL, varargin)
% %     THK = -pi/2 + acos((sin(TH3) + 2*sin(TH3 - TH4) + 6)/(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)) + acos((16*cos(TH4) + 48*sin(TH3) + 96*sin(TH3 - TH4) - 1)/(8*(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)));
%     PI = pi;
%     params1 = [ 4,     0,  2,      TH0; ...
%                 4,     0,  2,      TH1; ...
%                 0,  PI/2, 17,      TH2; ...
%                12,     0,  0,      TH3; ...
%                HH,     0,  0, THK-PI/2; ...
%                HL, -PI/2,  0, 	  PI/2];    %STATIC FRAME OF END EFFECTOR
%     X = FwdKinDH(params1);
% 
%     T0toT = cell(size(X,2),1);
%     T0toT(1) = X(1);
%     for c = 2:size(X,2)
%         T0toT(c) = {T0toT{c-1}*X{c}};
%        
%     end
%     
% %     listofpoints = zeros([1+size(T0toT,1), 6])
% %     for k = 1:size(T0toT,1)
% %         listofpoints(k+1,:) = extractPose(T0toT{k});
% %     end
% 
%     ee = cat(2, extractPose(T0toT{size(T0toT,1)}), THK);
%  
% end
% 
% function ee = fwdKinAlg12345(TH0, TH1, TH2, TH3, THK, HH, HL)
% %     THK = acos((sin(TH3)-2*sin(TH4-TH3)+6) ...
% %                     /sqrt(4*cos(TH4)+12*sin(TH3)-24*sin(TH4-TH3)+41)) ...
% %                     +acos((16*cos(TH4)+48*sin(TH3)-96*sin(TH4-TH3)-1) ...
% %                       /(8*sqrt(4*cos(TH4)+12*sin(TH3)-24*sin(TH4-TH3)+41)))-pi/2;
% %     THK = -pi/2 + acos((sin(TH3) + 2*sin(TH3 - TH4) + 6)/(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)) + acos((16*cos(TH4) + 48*sin(TH3) + 96*sin(TH3 - TH4) - 1)/(8*(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)));
% %     THK = THK-pi/3.38790991;
%     TZ = TH0+TH1+TH2;
%     TX = 0;
%     TY = TH3+THK;
%     
%     DT = 12*cos(TH3)+HL*cos(TH3+THK)+HH*sin(TH3+THK);
%     
%     X = 4*cos(TH0) + 4*cos(TH0+TH1) + DT*cos(TH0+TH1+TH2);
%     Y = 4*sin(TH0) + 4*sin(TH0+TH1) + DT*sin(TH0+TH1+TH2);
%     Z = 2+2+17+12*sin(TH3)+HL*sin(TH3+THK)+HH*cos(TH3+THK);
%     ee = [X,Y,Z,TX,TY,TZ,THK];
% end

