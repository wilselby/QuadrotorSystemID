% Wil Selby
% April 10, 2010
% Thesis

% Cascaded PID Control of a quadrotor

function [T,Y,XD] = sliding_control()

clear all; close all; clc;

global Kp Kd m d J Kfa Kft Jr B0 B1 B2 b V g a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 b1 b2 b3 motor_mat XD ctrl time sigma_bar aviobj index uplot;

Kp = 0.5;                           % lift coefficient (N*m/rad/s)
Kd = 0.01;                          % drag coefficient (N*m/rad/s)
m = 0.5;                            % mass (kg)
d = 0.25;                           % arm length (m)
J = diag([.005,.005,.010]);           % moment of inertia (kg*m^2)
Kfa = diag([5.56,5.56,5.56])*10^-4;   % aerodynamic friction coefficient
Kft = diag([.3729,.3712,.37]);   % translational drag coefficient
Jr = 2.8*10^-5;                     % mrotor inertia
B0 = 189.63;
B1 = 6.0612;
B2 = 0.012;
b = 280.19;
V = 12;                             % motor input (12V?)
g = 9.8;                            % gravity

motor_mat = [ Kp   Kp  Kp   Kp;
    -Kp   0   Kp   0;
    0   -Kp  0    Kp;
    Kd  -Kd  Kd  -Kd];

% Model parameters
a1 = (J(2,2) - J(3,3))/J(1,1);
a2 = -Kfa(1,1)/J(1,1);
a3 = -Jr/J(1,1);
b1 = d/J(1,1);

a4 = (J(3,3) - J(1,1))/J(2,2);
a5 = -Kfa(2,2)/J(2,2);
a6 = Jr/J(2,2);
b2 = d/J(2,2);

a7 = (J(1,1) - J(2,2))/J(3,3);
a8 = -Kfa(3,3)/J(3,3);
b3 = 1/J(3,3);

a9 = -Kft(1,1)/m;
a10 = -Kft(2,2)/m;
a11 = -Kft(3,3)/m;


sigma_bar = 0;

% State vector
% X = [roll,roll_d,pitch,pitch_d,yaw,yaw_d,x,x_d,y,y_d,z,z_d]

% Initial Conditions    %x     %y    %z
x_0 = [0; 0; 0; 0; 0; 0; 0.1; 0; -.2; 0; 1.1; 0];

% Control params
intg_pitch = 0;
intg_roll = 0;
intg_yaw = 0;
intg_thrust = 0;
last_t = 0;
x_vel_old = 0;
y_vel_old = 0;
prop_yaw = 0;
prop_thrust = 0;

% Simulation parameters
t_0 = 0;
t_f = 30.0;
step = 1e-1;
reltol = 1e-2;

perc = t_f/10;

r_des = 0;
p_des = 0;

index = 1;
uplot = [0 0 0];

% Set ODE step Size
options = odeset('MaxStep',step,'RelTol',reltol);

% Simulation
[T Y] = ode45(@odefun, [t_0 t_f], x_0, options);

%% X,Y,Z and yaw positions
figure(1);
subplot(2,2,1)
plot(T,Y(:,7));
hold on;
plot(time,XD(7,:),'r--');
title('X pos');
ylabel('Distance (m)');
xlabel('Time (sec)');
legend('Actual','Desired');

subplot(2,2,2);
plot(T,Y(:,9));
hold on;
plot(time,XD(9,:),'r--');
title('Y pos');
ylabel('Distance (m)');
xlabel('Time (sec)');
legend('Actual','Desired');

subplot(2,2,3);
plot(T,Y(:,5));
hold on;
plot(time,XD(5,:),'r--');
title('Yaw pos');
ylabel('Yaw Angle (rad)');
xlabel('Time (sec)');
legend('Actual','Desired');

subplot(2,2,4);
plot(T,Y(:,11));
hold on;
plot(time,XD(11,:),'r--');
title('Z pos');
ylabel('Distance (m)');
xlabel('Time (sec)');
legend('Actual','Desired');

%% Actual/Desired Roll and Pitch angles
% figure(2);
% subplot(2,1,1);
% plot(T,Y(:,1)*180/pi);
% hold on;
% plot(time,XD(1,:)*180/pi,'r--');
% title('Roll angle');
% ylabel('Angle (deg)');
% xlabel('Time (sec)');
% legend('Actual','Desired');
% 
% subplot(2,1,2);
% plot(T,Y(:,3)*180/pi);
% hold on;
% plot(time,XD(3,:)*180/pi,'r--');
% title('Pitch angle');
% ylabel('Angle (deg)');
% xlabel('Time (sec)');
% legend('Actual','Desired');

%% X and Y positions and inputs
% figure(10);
% subplot(2,2,1)
% plot(T,Y(:,7));
% hold on;
% plot(time,XD(7,:),'r--');
% title('X pos');
% ylabel('Distance (m)');
% xlabel('Time (sec)');
% legend('Actual','Desired');
% 
% subplot(2,2,2);
% plot(T,Y(:,9));
% hold on;
% plot(time,XD(9,:),'r--');
% title('Y pos');
% ylabel('Distance (m)');
% xlabel('Time (sec)');
% legend('Actual','Desired');
% 
% subplot(2,2,4);
% plot(T,Y(:,1)*180/pi);
% hold on;
% plot(time,XD(1,:)*180/pi,'r--');
% title('Roll angle');
% ylabel('Angle (deg)');
% xlabel('Time (sec)');
% legend('Actual','Desired');
% 
% subplot(2,2,3);
% plot(T,Y(:,3)*180/pi);
% hold on;
% plot(time,XD(3,:)*180/pi,'r--');
% title('Pitch angle');
% ylabel('Angle (deg)');
% xlabel('Time (sec)');
% legend('Actual','Desired');


%% Plot 3d positon
% figure();
% plot3(XD(7,:),XD(9,:),XD(11,:),'r--')
% hold on; 
% plot3(Y(:,7),Y(:,9),Y(:,11))
% title('3D Trajectory');

%% Plot error values
% figure()
% subplot(2,1,1);
% plot(sinroll);
% subplot(2,1,2);
% plot(sinpitch);
% 
% figure()
% % plot(uplot);
% subplot(3,1,1);plot(uplot(:,1));
% 
% subplot(3,1,2);plot(uplot(:,2));
% 
% subplot(3,1,3);plot(uplot(:,3));

%% Control Inputs
% figure();
% subplot(2,2,1);
% plot(time,ctrl(5,:)*180/pi);
% title('Ux');
% 
% subplot(2,2,2);
% plot(time,ctrl(6,:)*180/pi);
% title('Uy');
% 
% subplot(2,2,3);
% plot(time,ctrl(4,:));
% title('U yaw');
% 
% subplot(2,2,4);
% plot(time,ctrl(1,:));
% title('Uz');

%% Record movie
% aviobj = avifile('quad_sliding_control.avi');
% 
% %Display
% for k=1:size(T,1)
%     draw_quad(T(k),Y(k,:));
% %     pause(0.01);
% end
% 
% aviobj = close(aviobj);


%% Functions
    function X_d = odefun(t,x)
        
        if (mod(t,5) <= 1e-6)
                disp([num2str((t/t_f)*1e2),' % done']);
        end

        X_d = zeros(12,1);

        % Assign variables
        x(1);     % roll angle
        x(2);    % roll velocity
        x(3);     % pitch angle
        x(4);    % pitch velocity
        x(5);   % yaw angle
        x(6);  % yaw velocity
        x(7);   % x position
        x(8);    % x velocity
        x(9);     % y position
        x(10);  % y velocity
        x(11);   % z position
        x(12);  % z velocity


        xdes = trajectory(t,r_des,p_des);

        % [u,r_des,p_des] = control_slide2(t,x,xdes,sigma_bar);
        [u,r_des,p_des] = control_cpid(t,x,xdes,sigma_bar);
     

        ctrl(:,end+1) = u;

        U1 = u(1);
        U2 = u(2);
        U3 = u(3);
        U4 = u(4);
        Ux = u(5);
        Uy = u(6);


        % Calculate/insert accelerations into X_dot
        
        % simplified rotational dynamics
        a1=0;a2=0;a3=0;a4=0;a5=0;a6=0;a7=0;a8=0;
        
        X_d(1) = x(2);
        %X_d(2) = a1*x(4)*x(6) + a2*(x(2)^2) + a3*sigma_bar*x(4) + b1*U2;
        X_d(3) = x(4);
        %X_d(4) = a4*x(2)*x(6) + a5*(x(4)^2) + a6*sigma_bar*x(2) + b2*U3;
        X_d(5) = x(6);
        X_d(6) = a8*(x(6)) + b3*U4;  %a7*x(2)*x(4) + 
        X_d(7) = x(8);
        X_d(8) = a9*x(8) + Ux*U1/m;
        X_d(9) = x(10);
        X_d(10) = a10*x(10) + Uy*U1/m;
        X_d(11) = x(12);
        X_d(12) = a11*x(12) + (cos(x(1))*cos(x(3))*U1)/m - g;
        
        % Using Danny's model of attitude controller
        X_d(2) = -45.83*x(2) - 414.53*x(1) + 277.32*r_des;
        X_d(4) = -42.57*x(4) - 252.15*x(3) + 183.62*p_des;
        
        % Using estimated model
        
        X_d(8) = (8.5784*x(3) + 0)*1;
        X_d(10) = -(5.7964*x(1) + 0)*1;
        
%         kx= [5.7964    0.074337]; roll global x
%         ky= [8.5784    0.055125]; pitch global y

        time(end+1) = t;

    end

    function xd = trajectory(t,r_des,p_des)

        xd(1,:) = 0;
        xd(2,:) = 0;
        xd(3,:) = 0;
        xd(4,:) = 0;
        xd(5,:) = 0;
        xd(6,:) = 0;
        
        xd(8,:) = 0;
        
        xd(10,:) = 0;
        
        xd(12,:) = 0;
        
        xd(7,:) = 0;
        xd(9,:) = 0;
        xd(11,:) = 1;
        
%         if(t<=15)
%             xd(7,:) = 0;
%         end
%         if( t>15 && t <= 30)
%             xd(7,:) = .5;
%         end
%         if( t>30 && t <= 45)
%             xd(7,:) = .8;
%         end
%         if(t>45)
%             xd(7,:) = .6;
%         end
%         
%         if(t<=30)
%             xd(9,:) = 0;
%         end
%         if(t>30)
%             xd(9,:) = .5;
%         end
%         
%         if(t<=45)
%             xd(11,:) = 1.5;
%         end
%         if(t>45)
%             xd(11,:) = 1.25;
%         end
        
        % Vicon trajectory
%         if( t < 10)
%             xd(7,:) = 0;
%             xd(9,:) = 0;
%             xd(11,:) = 1.5;
%         else
%             xd(7,:) = 0.5*cos(2*pi*.1*(t-30)) - 0.5;
%             xd(9,:) = 0.5*sin(2*pi*.1*(t-30)) - 0.0;
%             xd(11,:) = 1.5;
%         end
        
        
%         xd(7,:) = 0; %sin(2*pi*t);
%         xd(9,:) = 0;    %sin(3*pi*t);
%         xd(11,:)= 1.5;

        XD(:,end+1) = xd;
    end

    function [u,r,p] = control_cpid(t,x,xd,sigma_bar)

%         Reference
%         x(1);     % roll angle
%         x(2);    % roll velocity
%         x(3);     % pitch angle
%         x(4);    % pitch velocity
%         x(5);   % yaw angle
%         x(6);  % yaw velocity
%         x(7);   % x position
%         x(8);    % x velocity
%         x(9);     % y position
%         x(10);  % y velocity
%         x(11);   % z position
%         x(12);  % z velocity
        
        % Control gains
        kp_pitch = 2;
        kd_pitch = 1;
        ki_pitch = 0;
        intg_min_pitch = -100;
        intg_max_pitch = 100;
        cmd_min_pitch = -1000;
        cmd_max_pitch = 1000;
        
        kp_roll = 2;
        kd_roll = 1;
        ki_roll = 0;
        intg_min_roll = -100;
        intg_max_roll = 100;
        cmd_min_roll = -1000;
        cmd_max_roll = 1000;
        
        kp_yaw = 2000;
        ki_yaw = 0;
        kd_yaw = 2.0;
        intg_min_yaw = -1000;
        intg_max_yaw = 1000;
        cmd_min_yaw = -1000;
        cmd_max_yaw = 1000;
        yaw_offset = 0;
        
        kp_thrust = 3.1;
        ki_thrust = 0.0;
        kd_thrust = 0.65;
        intg_min_thrust = -500;
        intg_max_thrust = 500;
        cmd_min_thrust = 0;
        cmd_max_thrust = 4000;
        thrust_offset = m*g;
        
        pos_err_min_pitch = -2500;
        pos_err_max_pitch = 2500;
        pos_err_min_roll = -2500;
        pos_err_max_roll = 2500;
        
        kp_vel_pitch = 1.25;
        kd_vel_pitch = .065;
        
        kp_vel_roll = 1.25;
        kd_vel_roll = .065;
        
        
        dt = t-last_t;
        if(dt)
            acc_x = (x(8)-x_vel_old)/dt;
            acc_y = (x(10)-y_vel_old)/dt;
        else
            acc_x = 0;
            acc_y = 0;
        end
        
        yaw = x(5);
        
        cos_yaw = cos(yaw);
        sin_yaw = sin(yaw);
        
        prop_pitch = cos_yaw*(xd(9)-x(9));
        prop_pitch = prop_pitch - sin_yaw*(xd(7)-x(7));
        diff_pitch = x(10);
        intg_pitch = intg_pitch + prop_pitch*dt;
        intg_pitch = minmax(intg_pitch,intg_min_pitch,intg_max_pitch);
        
        prop_roll = cos_yaw*(xd(7)-x(7));
        prop_roll = prop_roll + sin_yaw*(xd(9)-x(9));
        diff_roll = x(8);
        intg_roll = intg_roll + prop_roll*dt;
        intg_roll = minmax(intg_roll,intg_min_roll,intg_max_roll);
        
        pitch_err = 0;
        pitch_err = pitch_err + kp_pitch*prop_pitch;
        pitch_err = pitch_err + kd_pitch*diff_pitch;
        pitch_err = pitch_err + ki_pitch*intg_pitch;
        pitch_err = minmax(pitch_err,pos_err_min_pitch,pos_err_max_pitch);
        
        roll_err = 0;
        roll_err = roll_err + kp_roll*prop_roll;
        roll_err = roll_err + kd_roll*diff_roll;
        roll_err = roll_err + ki_roll*intg_roll;
        roll_err = minmax(roll_err,pos_err_min_roll,pos_err_max_roll);
        
        prop_pitch_vel = cos_yaw*(pitch_err-x(10));
        prop_pitch_vel = prop_pitch_vel - sin_yaw*(pitch_err-x(8));
        diff_pitch_vel = acc_y;
        
        prop_roll_vel = cos_yaw*(roll_err-x(8));
        prop_roll_vel = prop_roll_vel + sin_yaw*(roll_err-x(10));
        diff_roll_vel = acc_x;
        
        prop_prev = prop_yaw;
        prop_yaw = mod(xd(5)-x(5),2*pi);
        if(dt)
            diff_yaw = (prop_yaw - prop_prev)/dt;
        else
            diff_yaw = 0;
        end
        intg_yaw = intg_yaw + prop_yaw;
        intg_yaw = minmax(intg_yaw,intg_min_yaw,intg_max_yaw);
        
        prop_prev = prop_thrust;
        prop_thrust = xd(11)-x(11);
        if(dt)
            diff_thrust = (prop_thrust-prop_prev)/dt;
        else
            diff_thrust = 0;
        end
        intg_thrust = intg_thrust + prop_thrust;
        intg_thrust = minmax(intg_thrust,intg_min_thrust,intg_max_thrust);
        
        % Calculate commands
        cmd_pitch = kp_vel_pitch*prop_pitch_vel;
        cmd_pitch = cmd_pitch + kd_vel_pitch*diff_pitch_vel;
        cmd_pitch = minmax(cmd_pitch,cmd_min_pitch,cmd_max_pitch);
        
        cmd_roll = kp_vel_roll*prop_roll_vel;
        cmd_roll = cmd_roll + kd_vel_roll*diff_roll_vel;
        cmd_roll = minmax(cmd_roll,cmd_min_roll,cmd_max_roll);
        
        cmd_yaw = yaw_offset;
        cmd_yaw = cmd_yaw + kp_yaw*prop_yaw;
        cmd_yaw = cmd_yaw + kd_yaw*diff_yaw;
        cmd_yaw = cmd_yaw + ki_yaw*intg_yaw;
        cmd_yaw = minmax(cmd_yaw,cmd_min_yaw,cmd_max_yaw);
        
        cmd_thrust = thrust_offset;
        cmd_thrust = cmd_thrust + kp_thrust*prop_thrust;
        cmd_thrust = cmd_thrust + kd_thrust*diff_thrust;
        cmd_thrust = cmd_thrust + ki_thrust*intg_thrust;
        cmd_thrust = minmax(cmd_thrust,cmd_min_thrust,cmd_max_thrust);
        
        uplot(index,:) = [kp_thrust*prop_thrust,kd_thrust*diff_thrust,ki_thrust*intg_thrust];
        
        %Output variables
        u(1) = cmd_thrust;
        u(2) = 0; %not used
        u(3) = 0; %not used
        u(4) = cmd_yaw;
        u(5) = cos(x(1))*sin(x(3))*cos(x(5)) + sin(x(1))*sin(x(5)); %Ux, but just the roation matrix
        u(6) = cos(x(1))*sin(x(3))*sin(x(5)) - sin(x(1))*cos(x(5)); %Uy, but just the roation matrix
         
        % Desired roll and pitch angles for onboard controller, 2047*25 =
        % 51175 in 1/1000th of a degree => +-51.175 degrees range
        p = (cmd_roll*25)/1000;
        r = -(cmd_pitch*25)/1000;
        
        % Save for logging/plotting purposes
        XD(1,end) = xd(1);
        XD(3,end) = xd(3);
        
        
        % Update variables
        last_t = t;
        x_vel_old = x(8);
        y_vel_old = x(10);
        index = index+1;
        
        
    end
% ===============================================================
% This is the draw function
%================================================================
    function draw_quad(T,Y)
        global count x_plot z_plot x_final;

        persistent hFig;

        x = Y(7);
        z = Y(11);
        pitch = -Y(3);  % Matches system dynamics

        t = T;

        f_fig_bound = 3.5;
        r_fig_bound = -.5;
        t_fig_bound = 2.5;
        b_fig_bound = -0.5;


        base = .5;
        rotor_height = 0.05;
        blade_radius = .1;
        alpha = atan(rotor_height/base); % angle from cm to rotor
        h = base/cos(alpha);             % distace from cm to rotor

        f_base = [x + base/2*cos(pitch), z + base/2*sin(pitch)];
        r_base = [x - base/2*cos(pitch), z - base/2*sin(pitch)];

        f_rot = [f_base(1) - rotor_height*cos(pi/2-pitch), f_base(2) + rotor_height*sin(pi/2-pitch)];
        r_rot = [r_base(1) - rotor_height*cos(pi/2-pitch), r_base(2) + rotor_height*sin(pi/2-pitch)];

        f_blade_f = [ f_rot(1) + blade_radius*cos(pitch), f_rot(2) + blade_radius*sin(pitch)];
        f_blade_r = [ f_rot(1) - blade_radius*cos(pitch), f_rot(2) - blade_radius*sin(pitch)];

        r_blade_f = [ r_rot(1) + blade_radius*cos(pitch), r_rot(2) + blade_radius*sin(pitch)];
        r_blade_r = [ r_rot(1) - blade_radius*cos(pitch), r_rot(2) - blade_radius*sin(pitch)];


        if (isempty(hFig))
            hFig = figure(25);
            set(hFig,'DoubleBuffer','on');
        end

        figure(hFig);
        clf;

        % draw base
        line([r_base(1) f_base(1)],[r_base(2) f_base(2)]);
        hold on;
        % plot(f_base(1), f_base(2), 'ro');
        % plot(r_base(1), r_base(2), 'ro');

        % draw front rotor and blade
        line([f_base(1) f_rot(1)],[f_base(2) f_rot(2)]);
        line([f_blade_r(1) f_blade_f(1)],[f_blade_r(2) f_blade_f(2)]);

        % draw rear rotor and blade
        line([r_base(1) r_rot(1)],[r_base(2) r_rot(2)]);
        line([r_blade_r(1) r_blade_f(1)],[r_blade_r(2) r_blade_f(2)]);

        % draw ground
        line([r_fig_bound f_fig_bound],[0 0], 'Color', [.3 .5 .1],'LineWidth', 5);

        % draw wall
%         line([x_final x_final], [0 20],'Color',[.75 .6 .5]); %,'LineWidth', 2

        % draw trajectory
        x_plot(1,count) = x;
        z_plot(1,count) = z;
        plot(x_plot(1:end),z_plot(1:end),'r+','MarkerSize',3);
        count = count + 1;


        axis equal;
        axis([r_fig_bound f_fig_bound b_fig_bound t_fig_bound]);
        xlabel('Distance (m)');
        ylabel('Distance (m)');
        title(['time = ', num2str(t), ',  \phi = ', num2str(-pitch*180/pi), ' deg.']);

        drawnow;

        F = getframe(hFig);
        aviobj = addframe(aviobj,F);

    end

% Saturation function
    function x = minmax(x,minx,maxx)
        if(max(x)<minx)
            x = minx;
        end
        if(min(x)>maxx)
            x = maxx;
        end 
    end



end