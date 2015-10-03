%solve for LQR controller
A = [
    0     0     0     0     1     0     0 ;
    0     0     0     0     0     1     0 ;
    0     0     0     0     0     0     1 ;
    0     0     0     0     0     0     0 ;

    0     0     0     0     0     0     0 ;
    0     0     0     0     0     0     0 ;
    0     0     0     0     0     0     0 ;

    ];
B = [
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 kt(1);

    0 kx(1) 0 0;
    ky(1) 0 0 0;
    0 0 kz(1) 0;
    ];

Q = diag([3 3 3,10,1 1 2]);
R=diag([50,50,50,10]);

K = lqr(A,B,Q,R);

printMat(K,'K')

foo = K';
foo = foo(:)';
dstr=[];
for i = 1:length(foo)
    dstr = [dstr num2str(foo(i)) ', '];
end
disp('K=[')
disp(dstr);
disp(']');

dstr=[];
for i = 1:length(controlBias)
    dstr = [dstr num2str(controlBias(i)) ', '];
end
disp('bias=[')
disp(dstr);
disp(']');



