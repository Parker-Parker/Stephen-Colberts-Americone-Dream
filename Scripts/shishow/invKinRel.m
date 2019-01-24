function res = invKinRel(World, HH, HL)
% converging iterative jacobian
    TH0 = pi/2; TH1 = pi/2; TH2 = pi/4; TH3 = 0; TH4 = 0; 

    X = World(1);
    Y = World(2);
    Z = World(3);
    THX = World(4);
    THY = World(5);
    THZ = World(6);
    
    notconverged = true;
    while(notconverged)
%         ja = jacobGetSquare(TH0, TH1, TH2, TH3, TH4, HH, HL)
%         invja = inv(ja)
%         joints = invja*[X Y Z THY THZ]'
        joints = (jacobGetSquare(TH0, TH1, TH2, TH3, TH4, HH, HL))\[X Y Z THY THZ]'
        TH0 = joints(1); TH1 = joints(2); TH2 = joints(3); TH3 = joints(4); TH4 = joints(5); 
        fwdCheck = dofwdfast2(TH0, TH1, TH2, TH3, TH4, HH, HL)
        notconverged = sumabs(fwdCheck-World) > 0.00001
    end

    
end



function sum = sumabs(X)
    sum = 0;    
    for n = 1:size(X)
        sum = sum+abs(X(n));
    end

end