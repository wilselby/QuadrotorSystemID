function R = eulerToAxisAngle(euler)
R = quaternionToAxisAngle(eulerToQuaternion(euler));
end