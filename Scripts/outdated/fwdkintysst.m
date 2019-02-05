
clear;
% ins = mat2cell([...
%     pi/8, pi/8, pi/8, 0*pi/180, -18*pi/180 ...
%     , 1, 2],[1],[1 1 1 1 1 1 1]);
d = pi/180;
ins = num2cell([...
    
    0 0 0 0 0 1 2 ...
    
    ].*[d d d d d 1 1]);
ee1 = drawfwd(ins{:})-fwdKinAlg(ins{:})

x = 1:360
ty = zeros(1,360)
z = zeros(1,360)
for n = 1:360
    try
        ins = num2cell([0 0 0 n 120 1 2].*[d d d d d 1 1]);
%         xxx = (drawfwd(ins{:})-fwdKinAlg(ins{:}))
        xxx = (dofwdfast(ins{:})-fwdKinAlg(ins{:}))
        ty(n) = xxx(5);
        z(n) = xxx(3);
    end
end
plot(x,ty,x,z)