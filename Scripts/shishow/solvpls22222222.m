clear
% % % syms p2x p2y dt t0 t1 tz
% % % % eq1 = 4*cos(t0)+4*cos(t0+t1) == x - dt*cos(tz)
% % % % eq2 = 4*sin(t0)+4*sin(t0+t1) == y - dt*sin(tz)
% % % eq1 = 4*cos(t0)+4*cos(t0+t1) == p2x;
% % % eq2 = 4*sin(t0)+4*sin(t0+t1) == p2y;
% % % outsss = solve(eq1, eq2, t0, t1);

t = num2cell([.3 .3 .3 .3 .7]);
[TH0, TH1, TH2, TH3, TH4] = deal(t{:});     THK = -pi/2 + acos((sin(TH3) + 2*sin(TH3 - TH4) + 6)/(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)) + acos((16*cos(TH4) + 48*sin(TH3) + 96*sin(TH3 - TH4) - 1)/(8*(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)));
dt = 12*cos(TH3) + 1*sin(TH3+THK) + 2*cos(TH3+THK)

HH = 1
HL = 2

points = zeros(7,7)
points(1,:) = [0 1 0 0 0 0 0]
points(2,:) = [4*cos(TH0), 4*sin(TH0), 2, 0, 0, TH0, THK] + points(1,:)
points(3,:) = [4*cos(TH0+TH1), 4*sin(TH0+TH1), 2, 0, 0, TH1, 0] + points(2,:)
points(4,:) = [0, 0, 17, 0, 0, TH2, 0] + points(3,:)
points(5,:) = [12*cos(TH3)*cos(TH0+TH1+TH2), 12*cos(TH3)*sin(TH0+TH1+TH2), 12*sin(TH3), 0, -TH3, 0, 0] + points(4,:)
points(6,:) = [HH*sin(TH3+THK)*cos(TH0+TH1+TH2), HH*sin(TH3+THK)*sin(TH0+TH1+TH2), -HH*cos(TH3+THK), 0, pi/2-THK, 0, 0] + points(5,:)
points(7,:) = [HL*cos(TH3+THK)*cos(TH0+TH1+TH2), HL*cos(TH3+THK)*sin(TH0+TH1+TH2), HL*sin(TH3+THK), 0, -pi/2, 0, 0] + points(6,:)



% dofwdfast(T0, T1, T2, T3, T4, 1, 2);
[h, k] = fwdMats(TH0, TH1, TH2, TH3, TH4, HH, HL)

F = [k(7,1:3);(k(7,1:3)+h{6}(1:3,1)')];
G = eul2rotm(points(7,6:-1:4));
G = [points(7,1:3);(points(7,1:3)+G(1:3,1)')];

plot3(k(:,1), k(:,2), k(:,3), points(:,1), points(:,2), points(:,3), F(:,1), F(:,2), F(:,3), G(:,1), G(:,2), G(:,3)) 
axis auto; axis equal;  



% X = k(5:6,1:3)
% pdist(X,'euclidean')
% 
% X = points(5:6,1:3)
% pdist(X,'euclidean')
i = 1
t = zeros(2,1)
for x = 0:pi/180:2*pi
    
    try
        [h, k] = fwdMats(TH0, TH1, TH2, TH3, x, HH, HL)
        ponts = ForwardExplicit(TH0, TH1, TH2, TH3, x, HH, HL)
        ty(i) = k(end, 5) - ponts(end, 5)
        t(i) = i
        

    end
    i = i+1
end
figure(3)
plot(t,ty)