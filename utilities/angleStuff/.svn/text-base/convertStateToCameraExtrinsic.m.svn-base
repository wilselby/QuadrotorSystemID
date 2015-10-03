function statePose_camera_aligned = convertStateToCameraExtrinsic(statePose_global,state2CamR,state2CamT)
%apply found transform to the state measurements
statePose_global_aligned = convertStateToGlobalExtrinsic(statePose_global,state2CamR,state2CamT);

statePose_camera_aligned = convertBetweenWorldAndCamera(statePose_global_aligned);

end

