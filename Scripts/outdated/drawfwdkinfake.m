clear; clc; close all;
tic
t = num2cell([0 0 0 0 0 1 2]);
[TH0, TH1, TH2, TH3, THK, HH, HL] = deal(t{:});


PI = pi;
params1 = [ 4,     0,  2,      TH0; ...
            4,     0,  2,      TH1; ...
            0,  PI/2, 17,      TH2; ...
           13,     0,  0,      TH3; ...
           HH,     0,  0, THK-PI/2; ...
           HL, -PI/2,  0, 	  PI/2];    %STATIC FRAME OF END EFFECTOR
X = FwdKinDH(params1);

T0toT = cell(size(X,2),1);
for c = 1:size(X,2)
    T0toT(c) = {eye(4)}
    for n = 1:c
        T0toT(c) = {T0toT{c}*X{n}}
    end
end
listofpoints = zeros([1+size(T0toT,1), 6])
for k = 1:size(T0toT,1)
    listofpoints(k+1,:) = extractPose(T0toT{k});
    
end


TH4 = pi/4

params2 = [ 2, 0, 0,    -PI/2; ...
            2, 0, 0, THK-PI/2; ...
            4, 0, 0, 	 TH4];    %STATIC FRAME OF END EFFECTOR
X2 = FwdKinDH(params2);

linkageTFs =  T0toT(1:4);
linkageTFs(5) =  {T0toT{4}*X2{2}};
linkageTFs(7) =  {T0toT{3}*X2{1}};
linkageTFs(6) =  {linkageTFs{7}*X2{3}};

for f = 1:size(linkageTFs,1)
    list2(f+1,:) = extractPose(linkageTFs{f});
    
end

% celldisp(X)
%X = cellfun(@simplify,FwdKinDH(params1),'un',0)
% celldisp(T0toT)
toc
tic
plot3(listofpoints(:,1), listofpoints(:,2), listofpoints(:,3), list2(:,1), list2(:,2), list2(:,3))
% xlim([-5 15]);
% ylim([-15 10]);
axis equal;
toc