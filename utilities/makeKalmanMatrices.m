syms T Tsq_2 Kpx Kry

A =[
    1 0 0 0 0   0   T 0 0 0 Tsq_2 0      0       %x
    0 1 0 0 0   0   0 T 0 0 0     Tsq_2  0       %y
    0 0 1 0 0   0   0 0 T 0 0     0      Tsq_2   %z
    0 0 0 1 0   0   0 0 0 T 0     0      0       %yaw
    0 0 0 0 0   0   0 0 0 0 0     0      0       %pitch 
    0 0 0 0 0   0   0 0 0 0 0     0      0       %roll
    0 0 0 0 0   0   1 0 0 0 T     0      0       %xdot
    0 0 0 0 0   0   0 1 0 0 0     T      0       %ydot
    0 0 0 0 0   0   0 0 1 0 0     0      T       %zdot
    0 0 0 0 0   0   0 0 0 0 0     0      0       %yawdot
    0 0 0 0 Kpx 0   0 0 0 0 0     0      0       %xdotdot     
    0 0 0 0 0   Kry 0 0 0 0 0     0      0       %ydotdot
    0 0 0 0 0   0   0 0 0 0 0     0      0       %zdotdot
    ];


printMat(A,'A');
% pretty(A)

W = [ 
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    1 0 0 0 0 0
    0 1 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 1 0 0 0
    0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 0 0 0 1
    ];

printMat(W,'W')

H = [
    1 0 0 0 0 0 0 0 0 0 0 0 0
    0 1 0 0 0 0 0 0 0 0 0 0 0
    0 0 1 0 0 0 0 0 0 0 0 0 0
    0 0 0 1 0 0 0 0 0 0 0 0 0
    0 0 0 0 1 0 0 0 0 0 0 0 0
    0 0 0 0 0 1 0 0 0 0 0 0 0
    ];

printMat(H,'H')

V = [
    1 0 0 0 0 0
    0 1 0 0 0 0
    0 0 1 0 0 0
    0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 0 0 0 1
    ];
printMat(V,'V');



syms sp sa;

R = [
    sp  0 0  0  0  0
    0  sp 0  0  0  0
    0  0  sp 0  0  0
    0  0  0  sa 0  0
    0  0  0  0  sa 0
    0  0  0  0  0  sa
];
printMat(R,'R');


syms apc tpc xypc zpc 
Q = [
    apc  0    0    0    0    0
    0    apc  0    0    0    0
    0    0    tpc  0    0    0
    0    0    0    xypc 0    0
    0    0    0    0    xypc 0
    0    0    0    0    0    zpc
];
printMat(Q,'Q');
