%%%$$$find them 2
listWorld = zeros(length(listpts2), 6);
for n = 1:length(listpts2)
    listWorld(n,:) = dofwdfast2(...
        listpts2(n,1),...
        listpts2(n,2),...
        listpts2(n,3),...
        listpts2(n,4),...
        listpts2(n,5), 1, 2);
end

scatter3(listWorld(:,1),listWorld(:,2),listWorld(:,3), 0)
axis equal
