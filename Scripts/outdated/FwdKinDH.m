
function TransMats = FwdKinDH(DHparams)
    %Takes in n by 4 mat of dh params returns list tf matrices
    % [a0 alpha0 d0 theta0; a1 alpha1 d1 theta1;... an alphan dn thetan]
    TransMats = {};
    for c = 1:size(DHparams, 1)
        TransMats(c) = {DHtoTFmat(DHparams(c,:))};
    end 
end