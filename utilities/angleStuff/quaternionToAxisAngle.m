function aa = quaternionToAxisAngle(q)

theta = 2*acos(q(1));
w = q(2:4)/sin(theta/2);

aa = w*theta;


end