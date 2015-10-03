function [xi, yi ] = projectWorldToImage(X,Y,Z,R,t,camera_matrix)

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


%         disp(['GLOBAL coordinates =' num2str([X, Y, Z])])


x = r0*X + r1*Y + r2*Z + t0;
y = r3*X + r4*Y + r5*Z + t1;
z = r6*X + r7*Y + r8*Z + t2;

xp1 = x/z;
yp1 = y/z;

xi = xp1*fx + cx;
yi = yp1*fy + cy;


end