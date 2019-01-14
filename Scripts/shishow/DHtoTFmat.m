function TFMat = DHtoTFmat(DHparams) 
%[a alpha d theta]
    t = num2cell(DHparams);
    [a al d th] = deal(t{:});
    
    TFMat = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th); ...
             sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th); ...
                   0,          sin(al),          cos(al),         d; ...
                   0,                0,                0,         1];

end