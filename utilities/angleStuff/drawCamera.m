function drawCamera(camPose,drawOrigin,handleNo,clearFig)
%%%%%%%%%%%%%%%%%%%% SHOW EXTRINSIC RESULTS %%%%%%%%%%%%%%%%%%%%%%%%

if nargin<2
    drawOrigin=true;
end

if nargin<3 || handleNo<0
    handleNo =gcf;
end

if nargin <4
    clearFig=true;
end

if size(camPose,2)~=6
    disp('invalid pose')
end

if ishandle(handleNo),
    figure(handleNo);
    [a,b] = view;
else
    figure(handleNo);
    a = 50;
    b = 20;
end;

if clearFig
    clf
end
hold on



fc =[
    655.9468
    657.4533
    ];
cc =[
    319.5000
    239.5000
    ];
nx =640;
ny =480;
dX = 29;
alpha_c = 0;

%%% Show the extrinsic parameters

IP = 2*dX*[1 -alpha_c 0;0 1 0;0 0 1]*[1/fc(1) 0 0;0 1/fc(2) 0;0 0 1]*[1 0 -cc(1);0 1 -cc(2);0 0 1]*[0 nx-1 nx-1 0 0 ; 0 0 ny-1 ny-1 0;1 1 1 1 1];
BASE = 2*(.9)*dX*([0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]);
IP = reshape([IP;BASE(:,1)*ones(1,5);IP],3,15);
POS = [[6*dX;0;0] [0;6*dX;0] [-dX;0;5*dX] [-dX;-dX;-dX] [0;0;-dX]];



hold on;

for camNo = 1:size(camPose,1)

    Tc_kk =  camPose(camNo,1:3)';
    omc_kk =camPose(camNo,4:6)';


    if ~isnan(omc_kk(1,1)),

        R_kk = rodrigues(omc_kk);

        BASEk = R_kk'*(BASE - Tc_kk * ones(1,6));
        IPk = R_kk'*(IP - Tc_kk * ones(1,15));
        POSk = R_kk'*(POS - Tc_kk * ones(1,5));

        p1 = struct('vertices',IPk','faces',[1 4 2;2 4 7;2 7 10;2 10 1]);
        h1 = patch(p1);
        set(h1,'facecolor',[52 217 160]/255,'EdgeColor', 'r');
        p2 = struct('vertices',IPk','faces',[1 10 7;7 4 1]);
        h2 = patch(p2);
        %set(h2,'facecolor',[236 171 76]/255,'EdgeColor', 'none');
        set(h2,'facecolor',[247 239 7]/255,'EdgeColor', 'none');

        %     plot3(BASEk(1,:),BASEk(2,:),BASEk(3,:),'b-','linewidth',1');
        plot3(IPk(1,:),IPk(2,:),IPk(3,:),'r-','linewidth',1);



    end
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

if drawOrigin
    drawPose(zeros(1,6),0,-1,0);
end

end
