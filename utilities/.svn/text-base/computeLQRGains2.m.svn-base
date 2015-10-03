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

    kx(1) 0 0 0;
    0 ky(1) 0 0;
    0 0 kz2(1) 0;
    ];

Q = diag([40 40 3, 20, 5 5 2]);

R = diag([10, 10, 50, 10]);

K = lqr(A,B,Q,R);

printMat(K,'K')

foo = K';
foo = foo(:)';
disp('K=[')
for i = 1:length(foo)
    fprintf(1, '%.10f, ',foo(i));
end
disp(' ');
disp(']');

disp('bias=[')
for i = 1:length(controlBias)
    fprintf(1, '%.10f, ',controlBias(i));
end
disp(' ');
disp(']');


disp('batteryGain=[')
fprintf(1, '%.10f, ',batteryGain);
disp(' ');
disp(']');


