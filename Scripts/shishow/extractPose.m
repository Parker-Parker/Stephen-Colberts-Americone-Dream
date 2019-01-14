function pose = extractPose(tmat)
%tmat = 4x4 matrix

thy = -asin(tmat(3,1));
if tmat(3,1)== -1
    thz = 0;
    thx = -thz + atan2(-tmat(1,2), -tmat(1,3));
elseif tmat(3,1)== 1
    thz = 0;
    thx = thz + atan2(tmat(1,2), tmat(1,3));
else
    thx = atan2(tmat(3,2)/cos(thy), tmat(3,3)/cos(thy));
    thz = atan2(tmat(2,1)/cos(thy), tmat(1,1)/cos(thy));
end

pose = cat(2,tmat(1:3,4)', thx, thy, thz);
end