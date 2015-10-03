function animateTrajectory(x)
fig = figure(26);
set(fig,'DoubleBuffer','on','name','quadSimulation');
cla;
axis equal;
axis([-2 2 -2 2 0 2]);

for i = 1:size(x,1)
    aa =  eulerToAxisAngle(x(i,4:6));
    quadPose=[x(i,1:3)';aa];
    draw(quadPose)
end


    function draw(quadPose,t)
        if nargin <2
            t=0;
        end

        cla;
        hold on;

        origin = .1*(...
            [0 1 0 0 0 0;
            0 0 0 1 0 0;
            0 0 0 0 0 1]);



        plot3(origin(1,1:2),origin(2,1:2),origin(3,1:2),'r-','linewidth',2');
        plot3(origin(1,3:4),origin(2,3:4),origin(3,3:4),'b-','linewidth',2');
        plot3(origin(1,5:6),origin(2,5:6),origin(3,5:6),'g-','linewidth',2');


        QUAD = .3*(...
            [-1 0 0 1 0 0 0 0;
            0 0 0 0 -1 1 0 0;
            0 0 0 0 0 0 0 .3]);

        QUAD = rodrigues(quadPose(4:6))*(QUAD)+ quadPose(1:3) * ones(1,size(QUAD,2));

        plot3(QUAD(1,1:2),QUAD(2,1:2),QUAD(3,1:2),'b-','linewidth',5');
        plot3(QUAD(1,3:4),QUAD(2,3:4),QUAD(3,3:4),'r-','linewidth',5');
        plot3(QUAD(1,5:6),QUAD(2,5:6),QUAD(3,5:6),'b-','linewidth',5');
        plot3(QUAD(1,7:8),QUAD(2,7:8),QUAD(3,7:8),'g-','linewidth',5');



        hold off
        title(['t = ', num2str(t)]);
        drawnow;
    end %drawing func

end
