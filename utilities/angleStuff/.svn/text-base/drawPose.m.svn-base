function drawPose(pose,drawOrigin,handleNo,clearFig)


if nargin<2
    drawOrigin=true;
end

if nargin<3 || handleNo<0
    handleNo =gcf;
end

if nargin <4
    clearFig=true;
end

if size(pose,2)~=6
    disp('invalid pose')
    keyboard;
end

if ishandle(handleNo),
    figure(handleNo);
    [a,b] = view;
else
    figure(handleNo);
    a = 50;
    b = 20;
end;


if drawOrigin
    pose = [zeros(1,6);pose];
end

if clearFig
    clf
end
hold on

dX = 20;
BASE = 2*(.9)*dX*([0 1 0 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 0 1]);

for camNo = 1:size(pose,1)

    BASEk = rodrigues(pose(camNo,4:6))*(BASE)+ pose(camNo,1:3)' * ones(1,6);

    % plot3(BASEk(1,:),BASEk(2,:),BASEk(3,:),'b-','linewidth',1');


    plot3(BASEk(1,1:2),BASEk(2,1:2),BASEk(3,1:2),'b-','linewidth',1');
    plot3(BASEk(1,3:4),BASEk(2,3:4),BASEk(3,3:4),'r-','linewidth',1');
    plot3(BASEk(1,5:6),BASEk(2,5:6),BASEk(3,5:6),'g-','linewidth',1');

end



figure(handleNo);rotate3d on;
axis('equal');
title('aligned Vs Extrinsic');
xlabel('X_{world}')
ylabel('Y_{world}')
zlabel('Z_{world}')
view(a,b);
axis vis3d;
axis tight;
grid on;

hold off;
set(handleNo,'color',[1 1 1]);


figure(handleNo);
rotate3d on;

end
