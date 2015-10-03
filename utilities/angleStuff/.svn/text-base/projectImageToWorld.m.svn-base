function [X, Y] = projectImageToWorld(xi,yi,R,t,camera_matrix)
r0 = R(1,1);
r1 = R(1,2);
r2 = R(1,3);
r3 = R(2,1);
r4 = R(2,2);
r5 = R(2,3);
r6 = R(3,1);
r7 = R(3,2);
r8 = R(3,3);

t0 = t(1);
t1 = t(2);
t2 = t(3);

fx = camera_matrix(1,1);
fy = camera_matrix(2,2);
cx = camera_matrix(1,3);
cy = camera_matrix(2,3);


xp = (xi-cx)/fx;
yp = (yi-cy)/fy;

X = (xp*t2*r4-r7*xp*t1+r1*t1-t0*r4+t0*yp*r7-r1*yp*t2)/(-xp*r6*r4-r0*yp*r7+r0*r4+yp*r6*r1-r3*r1+r3*xp*r7);

Y = (xp*r6*t1-r3*xp*t2+r0*yp*t2-r0*t1-yp*r6*t0+r3*t0)/(-xp*r6*r4-r0*yp*r7+r0*r4+yp*r6*r1-r3*r1+r3*xp*r7);

end