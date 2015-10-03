function globalStates = convertBodyToGlobal(bodyStates)

globalStates = bodyStates;
for i = 1:size(bodyStates,1)
    R = rotMat2D(bodyStates(i,4));
    globalStates(i,1:2) =        R*bodyStates(i,1:2)';
    
    if size(bodyStates,2)>=8
        globalStates(i,7:8) =    R*bodyStates(i,7:8)';
    end
    if size(bodyStates,2)>=14
        globalStates(i,13:14) =  R*bodyStates(i,13:14)';
    end
end

end