function plotCameras(ext)
%%%%%%%%%%%%%%%%%%%% SHOW EXTRINSIC RESULTS %%%%%%%%%%%%%%%%%%%%%%%%

if ~exist('show_camera'),
    show_camera = 1;
end;


no_grid = 0;


if ~exist('alpha_c'),
    alpha_c = 0;
end;




% Color code for each image:

colors = 'brgkcm';


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




%%% Show the extrinsic parameters
XX_kk =[0,29,58,87,116,145,0,29,58,87,116,145,0,29,58,87,116,145,0,29,58,87,116,145,0,29,58,87,116,145,0,29,58,87,116,145,0,29,58,87,116,145,0,29,58,87,116,145;203,203,203,203,203,203,174,174,174,174,174,174,145,145,145,145,145,145,116,116,116,116,116,116,87,87,87,87,87,87,58,58,58,58,58,58,29,29,29,29,29,29,0,0,0,0,0,0;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;];

IP = 2*dX*[1 -alpha_c 0;0 1 0;0 0 1]*[1/fc(1) 0 0;0 1/fc(2) 0;0 0 1]*[1 0 -cc(1);0 1 -cc(2);0 0 1]*[0 nx-1 nx-1 0 0 ; 0 0 ny-1 ny-1 0;1 1 1 1 1];
BASE = 2*(.9)*dX*([0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]);
IP = reshape([IP;BASE(:,1)*ones(1,5);IP],3,15);
POS = [[6*dX;0;0] [0;6*dX;0] [-dX;0;5*dX] [-dX;-dX;-dX] [0;0;-dX]];


if ishandle(4),
    figure(4);
    [a,b] = view;
else
    figure(4);
    a = 50;
    b = 20;
end;


figure(4);
clf;
hold on;

n_ima = size(ext,1);
for kk = 1:n_ima,

    Tc_kk = ext(kk,4:6)';
    omc_kk = ext(kk,1:3);


    if ~exist(['n_sq_x_' num2str(kk)]),
        no_grid = 1;
    else
        eval(['n_sq_x = n_sq_x_' num2str(kk) ';']);
        if isnan(n_sq_x(1)),
            no_grid = 1;
        end;
    end;


    n_sq_x = 8;
    n_sq_y = 7;

    if ~isnan(omc_kk(1,1)),

        R_kk = rodrigues(omc_kk);

        BASEk = R_kk'*(BASE - Tc_kk * ones(1,6));
        IPk = R_kk'*(IP - Tc_kk * ones(1,15));
        POSk = R_kk'*(POS - Tc_kk * ones(1,5));

        YY_kk = XX_kk;

        if ~no_grid,

            YYx = zeros(n_sq_x+1,n_sq_y+1);
            YYy = zeros(n_sq_x+1,n_sq_y+1);
            YYz = zeros(n_sq_x+1,n_sq_y+1);

            YYx(:) = YY_kk(1,:);
            YYy(:) = YY_kk(2,:);
            YYz(:) = YY_kk(3,:);

            figure(4);

            if show_camera,

                p1 = struct('vertices',IPk','faces',[1 4 2;2 4 7;2 7 10;2 10 1]);
                h1 = patch(p1);
                set(h1,'facecolor',[52 217 160]/255,'EdgeColor', 'r');
                p2 = struct('vertices',IPk','faces',[1 10 7;7 4 1]);
                h2 = patch(p2);
                %set(h2,'facecolor',[236 171 76]/255,'EdgeColor', 'none');
                set(h2,'facecolor',[247 239 7]/255,'EdgeColor', 'none');

                plot3(BASEk(1,:),BASEk(2,:),BASEk(3,:),'b-','linewidth',1');
                plot3(IPk(1,:),IPk(2,:),IPk(3,:),'r-','linewidth',1);
                text(POSk(1,5),POSk(2,5),POSk(3,5),num2str(kk),'fontsize',10,'color','k','FontWeight','bold');

            end;

            hhh= mesh(YYx,YYy,YYz);
            set(hhh,'edgecolor',colors(rem(kk-1,6)+1),'linewidth',1); %,'facecolor','none');

        else

            figure(4);

            if show_camera,

                p1 = struct('vertices',IPk','faces',[1 4 2;2 4 7;2 7 10;2 10 1]);
                h1 = patch(p1);
                set(h1,'facecolor',[52 217 160]/255,'EdgeColor', 'r');
                p2 = struct('vertices',IPk','faces',[1 10 7;7 4 1]);
                h2 = patch(p2);
                %set(h2,'facecolor',[236 171 76]/255,'EdgeColor', 'none');
                set(h2,'facecolor',[247 239 7]/255,'EdgeColor', 'none');

                plot3(BASEk(1,:),BASEk(2,:),BASEk(3,:),'b-','linewidth',1');
                plot3(IPk(1,:),IPk(2,:),IPk(3,:),'r-','linewidth',1);
                hww = text(POSk(1,5),POSk(2,5),POSk(3,5),num2str(kk),'fontsize',10,'color','k','FontWeight','bold');

            end;

            plot3(YY_kk(1,:),YY_kk(2,:),YY_kk(3,:),['.' colors(rem(kk-1,6)+1)]);


        end;

    end;

end;

figure(4);rotate3d on;
axis('equal');
title('Extrinsic parameters (world-centered)');
%view(60,30);
xlabel('X_{world}')
ylabel('Y_{world}')
zlabel('Z_{world}')
view(a,b);
axis vis3d;
axis tight;
grid on;
plot3(3*dX*[1 0 0 0 0],3*dX*[0 0 1 0 0],3*dX*[0 0 0 0 1],'r-','linewidth',3);
hold off;
set(4,'color',[1 1 1]);

set(4,'Name','3D','NumberTitle','off');

%hh = axis;
%hh(5) = 0;
%axis(hh);

%fprintf(1,'To generate the complete movie associated to the optimization loop, try: check_convergence;\n');

figure(4);
rotate3d on;