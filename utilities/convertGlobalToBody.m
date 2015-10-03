function bodyStates = convertGlobalToBody(globalStates)

bodyStates = globalStates;
for i = 1:size(globalStates,1)
    R = rotMat2D(-globalStates(i,4));
    bodyStates(i,1:2) =         R*globalStates(i,1:2)';
    
    if size(globalStates,2)>=8
        bodyStates(i,7:8) =     R*globalStates(i,7:8)';
    end
    if size(globalStates,2)>=14
        bodyStates(i,13:14) =   R*globalStates(i,13:14)';
    end
end

end