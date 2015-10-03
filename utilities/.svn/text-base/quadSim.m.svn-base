function [trajectory dotTrajectory uTrajectory] = quadSim(s)
% input:
%  s              initial conditions for simulation
%
% outputs:
%

global stopper
stopper =false;

if nargin<1
    %  Default Initial Conditions
    %     [x y z theta xdot ydot zdot]  NO theta dot, because we don't
    s = [1.5; 1.5; .2; .5;   0; 0; 0];
end

kx= [5.2285    0.062423];
ky= [-5.2709    -0.27075];
kz= [7.4501     -3.8533];
kt= [1.5465    0.032137];
 
kp= [-0.66403    0.010099];
kr= [0.63779    0.021683];
controlBias= [-0.012032   -0.051937     0.51777   -0.018763];

% dt = .0087;%5e-4;
dt = 0.0521; % discrete simulation step size
display_dt = 0.01;  % change the speed of the drawing

t=0;
createDrawing(s);
last_display_t = 0;


disp('Hit the "Stop" button to finish');
numStores=floor(30/dt);
trajectory = zeros(numStores,length(s)+1);
dotTrajectory= zeros(numStores,length(s)+1);
uTrajectory = zeros(numStores,4+1);
stepCount = 0;
tstart = clock;
realtime= false;
for i = 1:numStores;
    % while (~stopper)
    stepCount = stepCount+1;

    %     u = u(stepCount,:);
    y = s+randn(size(s)).*0;
    u = control(y,[-.5;1;1.5;0;0;0;0]);
    uTrajectory(stepCount,:) = [u',t];
    sdot =dynamics(s,u);
    dotTrajectory(stepCount,:) = [sdot',t];

    s = s + dt*sdot;
    t = t+dt;
    if realtime
        rt = etime(clock,tstart);
        if t-rt>.1
            %             disp(['t = ' num2str(t) ' rt = ' num2str(rt) ' pausing for ' num2str(t-rt) ' secs'])
            pause(t-rt)
        end
    end
    trajectory(stepCount,:) = [s',t];

    if (t - last_display_t > display_dt)

        yaw = s(4);
        pitch = u(1)*kp(1);%+kp(2);
        roll = u(2)*kr(1);%+kr(2);
        aa =  eulerToAxisAngle([yaw pitch roll]);
        quadPose=[s(1:3);aa];


        draw(quadPose,t);
        last_display_t = t;
    end

    if stopper
        break;
    end


end
trajectory = trajectory(1:stepCount,:);
dotTrajectory = dotTrajectory(1:stepCount,:);
uTrajectory = uTrajectory(1:stepCount,:);
figure(26)
close(26)
figure(26)
subplot(2,1,1);
plot(trajectory(:,end),trajectory(:,1:4));
subplot(2,1,2);
plot(uTrajectory(:,end),uTrajectory(:,1:4));


    function u = control(s,g)
        if nargin <2
            g = zeros(size(s));
        end

        %         convert goal from world to body coordinates
        bg = g;
        bg(1:2) = rotMat2D(s(4))*[g(2);g(1)];

       K=[
0.1,0,0,0,0.21966,0,0,
0,-0.1,8.8424e-18,0,0,-0.21896,-1.544e-18,
0,3.5126e-17,0.1,0,0,2.1823e-18,0.19195,
0,0,0,0.1,0,0,0,
];
        x = s-bg;
        u=-K*x;

        u = u +controlBias'; %add in bias terms

        %     u = min(u,1);
        %     u = max(u,-1);

    end

    function sdot = dynamics(s,u)
        if nargin <2
            u=0;
        end

        %         xdot = s(5);
        %         ydot = s(6);
        %         zdot = s(7);
        %         tdot = kt(1)*u(4)+kt(2);
        %
        %
        %         xdotdot = kx(1)*u(1)+kx(2);
        %         ydotdot = ky(1)*u(2)+ky(2);
        %         zdotdot = kz(1)*u(3)+kz(2);
        %         tdotdot = 0;
        %
        %         sdot2 = [xdot;ydot;zdot;tdot; xdotdot;ydotdot;zdotdot];

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
            0 0 kz(1) 0;
            ];

        offset = [
            0;
            0;
            0;
            kt(2);

            kx(2);
            ky(2);
            kz(2);
            ];

        sdot = A*s+B*u+offset;


    end %dynamics func


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

    function createDrawing(x)
        figure(25);
        close(25);
        fig = figure(25);
        set(fig,'DoubleBuffer','on','name','quadSimulation');
        uicontrol('Style', 'pushbutton', 'String', 'Stop','Callback',@stopButton_Callback);
        cla;
        axis equal;
        axis([-2 2 -2 2 0 2]);
        draw(x,0);
    end
end % main func

function stopButton_Callback(hObject, eventdata, handles)
global stopper
disp('Stopping the Simulation');
stopper = true;
end

