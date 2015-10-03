function statePose_global_aligned = convertStateToGlobalExtrinsic(statePose_global,state2CamR,state2CamT)
%apply found transform to the state measurements
statePose_global_aligned = zeros(size(statePose_global));
for i = 1:size(statePose_global,1)
    Rs = rodrigues(statePose_global(i,4:6));
    statePose_global_aligned(i,1:3) = (statePose_global(i,1:3)'+Rs'*state2CamT')';
    try
        statePose_global_aligned(i,4:6) = rodrigues(Rs*state2CamR)';
    catch
        statePose_global_aligned(i,4:6) = statePose_global_aligned(i-1,4:6);
        i
        disp('Had to use prev!!!!')
    end
end

end

