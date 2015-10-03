function R = eulerToMatrix(euler)
R = quaternionToMatrix(eulerToQuaternion(euler));
end