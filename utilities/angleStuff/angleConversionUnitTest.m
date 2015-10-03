sum=0;
for i =1:100
    aa = randn(3,1)*pi/2;
    if norm(aa)>1
        continue;
    end
    r = eulerToAxisAngle(axisAngleToEuler(quaternionToAxisAngle(axisAngleToQuaternion(rodrigues(eulerToMatrix(quaternionToEuler(eulerToQuaternion(axisAngleToEuler(aa)))))))));
    sum = sum + norm(r-aa);
    
% eulerToQuaternion.m	  
% quaternionToEuler.m
% axisAngleToEuler.m	  
% quaternionToMatrix.m
% axisAngleToQuaternion.m   
% drawPose.m		
% eulerToMatrix.m	
% quaternionToAxisAngle.m


end
sum