function quaternion = axisAngleToQuaternion(axis_angle)
quaternion = zeros(1,4);
theta = norm(axis_angle);
if theta ~=0
    quaternion(1) = cos(theta/2);
    quaternion(2) = axis_angle(1)/theta*sin(theta/2);
    quaternion(3) = axis_angle(2)/theta*sin(theta/2);
    quaternion(4) = axis_angle(3)/theta*sin(theta/2);
end

end