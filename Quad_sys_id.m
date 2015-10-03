% Wil Selby
% Washington, DC
% July 08, 2015

% This script process .mat files comprised of ArduPilot Mavlink messages
% and Vicon messages. These measurements are used to identify the
% quadrotor's system parameters.

% http://www.mathworks.com/matlabcentral/fileexchange/33341-quaternion-m
% http://www.mathworks.com/matlabcentral/fileexchange/48493-data-processing
% -for-modeling-with-the-nir-data/content/Functions/smooth.m
%% Initialization

clear all;
close all;
clc;

% Add Paths
addpath logs
addpath utilities
addpath C:\Users\test\Dropbox\UAS Research\MATLAB\SysID\utilities\Smooth
addpath C:\Users\test\Dropbox\UAS Research\MATLAB\SysID\utilities\angleStuff
addpath C:\Users\test\Dropbox\UAS Research\MATLAB\SysID\utilities\rls\matlab

% Load Log Data
% load data_1;    % Pitch 
% load data_2;    % Yaw
% load data_3;    % Z
load data_4;    % Multi

%% Find Start and End Times based on Thrust Command
ch3 = (double(pwm_in(3,:)));
t_start = 1;
t_end = size(ch3,2);

% Hovering is around 1425 PWM. Record data between first and last hover
i = 1;
while((ch3(i)<1425) && (i < size(ch3,2)))
    t_start = t_start+1;
    i = i+1;
end
i = 1;
while((ch3(end-i)<1425) && (i < size(ch3,2)))
    t_end = t_end-1;
    i = i+1;
end

t_start = 95;   % Hardcode for data_4 (time logging error)

%% ArduPilot/Mavlink - IMU

% Time (s)
imu_time = t_imu(1,t_start:t_end);

% Acceleromter (m/s^2)
x_dd = accel(1,t_start:t_end);  
y_dd = accel(2,t_start:t_end); 
z_dd = accel(3,t_start:t_end);  

% Gyroscope (rad/s)
gyro_p = gyro(1,t_start:t_end);   % + roll -> right
gyro_q = -gyro(2,t_start:t_end);  % + pitch -> back
gyro_r = -gyro(3,t_start:t_end);  % + yaw -> ccw

%% ArduPilot/Mavlink - RC In

% Time (s)
tinds = t_pwm_in >= t_imu(t_start) & t_pwm_in <= t_imu(t_end);
rc_time = t_pwm_in(tinds);

% RC values 1100-1900 (us)
ch1_raw = double(pwm_in(1,tinds));
ch2_raw = double(pwm_in(2,tinds));
ch3_raw = double(pwm_in(3,tinds));
ch4_raw = double(pwm_in(4,tinds));

rpy_max = 45;    % Max angle in degrees, high value->positive angle

% RC values transformed
ch1 = -(double(pwm_in(1,tinds))-1450)*(rpy_max/400)*(2*pi/360);      % Roll (desired angle in rad, reverse mapped in Mission Planner)
ch2 = -(double(pwm_in(2,tinds))-1500)*(rpy_max/400)*(2*pi/360);      % Pitch (desired angle in rad)
ch3 =  (double(pwm_in(3,tinds))-1425)/100;                           % Thrust (pos z down)
ch4 =  -(double(pwm_in(4,tinds))-1525)*(rpy_max/400)*(2*pi/360);      % Yaw Rate (desired rad/s, reverse mapped in Mission Planner)

%% ArduPilot/Mavlink - PWM Out

% Time (s)
tinds = t_pwm_out >= t_imu(t_start) & t_pwm_out <= t_imu(t_end);
pwm_time = t_pwm_out(tinds);

max_v = 12.6;   % Max motor voltage (3s LiPo)
Kv = 700;       % Motor velocity constant (RPM/V estimated)

% ESC PWM commands (us, 1000-2000)

m1_raw = pwm_out(3,tinds);  % Front motor 
m2_raw = pwm_out(1,tinds);  % Right motor 
m3_raw = pwm_out(4,tinds);  % Rear motor 
m4_raw = pwm_out(2,tinds);  % Left motor 

% Estimated propeller speed (rad/s)
m1 = double(m1_raw-1000)*(max_v/1000)*Kv*(2*pi/60);  % Front motor speed 
m2 = double(m2_raw-1000)*(max_v/1000)*Kv*(2*pi/60);  % Right motor speed
m3 = double(m3_raw-1000)*(max_v/1000)*Kv*(2*pi/60);  % Rear motor speed 
m4 = double(m4_raw-1000)*(max_v/1000)*Kv*(2*pi/60);  % Left motor speed

%% Vicon Data

% Time (s)
tinds = t_att >= t_imu(t_start) & t_att <= t_imu(t_end);
vicon_time = t_att(tinds); 

% Position (Vicon pos: Z up, Y left, X fwd m)
Xg = pos(1,tinds);
Yg = -pos(2,tinds);     % Convert to pos Y right
Zg = -pos(3,tinds);      % Convert to pos Z down

% Attitude (x,y,z,w quaternions)
qx = att(1,tinds);
qy = att(2,tinds);
qz = att(3,tinds);
qw = att(4,tinds);

% Convert to Euler angles
q = quaternion(qw, qx, qy, qz);
angles = EulerAngles( q, '123' );   % Radians

% Vicon Euler angles (rad)
roll_v = angles(1,:);
pitch_v = -angles(2,:);
yaw_v = -angles(3,:);

%% Update Time Vectors

rc_time = rc_time - imu_time(1);
pwm_time = pwm_time - imu_time(1);
vicon_time = vicon_time - imu_time(1);
imu_time = imu_time - imu_time(1);

%% Make Mavlink Messages Same Size
if(size(imu_time,2)<size(rc_time,2))
    for i=1:size(rc_time,2)-size(imu_time,2)
        imu_time(end+1)=imu_time(end)+(imu_time(end)-imu_time(end-1));
        
        % Acceleromter
        x_dd(end+1) = x_dd(end)+(x_dd(end)-x_dd(end-1));
        y_dd(end+1) = y_dd(end)+(y_dd(end)-y_dd(end-1));
        z_dd(end+1) = z_dd(end)+(z_dd(end)-z_dd(end-1));
        
        % Gyroscope
        gyro_p(end+1) = gyro_p(end)+(gyro_p(end)-gyro_p(end-1));
        gyro_q(end+1) = gyro_q(end)+(gyro_q(end)-gyro_q(end-1));
        gyro_r(end+1) = gyro_r(end)+(gyro_r(end)-gyro_r(end-1));
    end
elseif(size(imu_time,2)>size(rc_time,2))
    for i=1:size(imu_time,2)-size(rc_time,2)
        imu_time = imu_time(1:end-1);
        
        % Acceleromter
        x_dd = x_dd(:,1:end-1);
        y_dd = y_dd(:,1:end-1);
        z_dd = z_dd(:,1:end-1);
        
        % Gyroscope
        gyro_p = gyro_p(:,1:end-1);
        gyro_q = gyro_q(:,1:end-1);
        gyro_r = gyro_r(:,1:end-1);
    end
end

%% Differentiate & Filter

viconPer = median(diff(vicon_time));    % (s) ~100Hz
apmPer = median(diff(rc_time));         % (s) ~10Hz

% X axis derivatives
gxv = meanFilter(diff(Xg)',40)./viconPer;   % m/s
gxa = meanFilter(diff(gxv),40)./viconPer;   % m/s^2
gxv = [0; gxv];
gxa = [0; 0; gxa];
gxa([1:40,end-40:end])=0; %get rid of end effects...

% Y axis derivatives
gyv = meanFilter(diff(Yg)',40)./viconPer;   % m/s
gya = meanFilter(diff(gyv),40)./viconPer;   % m/s^2
gyv = [0; gyv];
gya = [0; 0; gya];
gya([1:40,end-40:end])=0; %get rid of end effects...

% Z axis derivatives
gzv = meanFilter(diff(Zg)',10)./viconPer;   % m/s
gza = meanFilter(diff(gzv),10)./viconPer;   % m/s^2
gzv = [0; gzv];
gza = [0; 0; gza];
gza([1:40,end-40:end])=0; %get rid of end effects...

% Global to Body Rotations
bv = convertGlobalToBody([gxv,gyv,gzv,yaw_v']);
bxv = bv(:,1);  % m/s
byv = bv(:,2);  % m/s

ba = convertGlobalToBody([gxa,gya,gza,yaw_v']);
bxa = ba(:,1);  % m/s^2
bya = ba(:,2);  % m/s^2

% Rotation 
rp = medianFilter(roll_v',10);          % rad
rv = meanFilter(diff(rp),40)./viconPer; % rad/s
ra = meanFilter(diff(rv),40)./viconPer; % rad/s^2
rv = [0; rv];
ra = [0; 0; ra];
ra([1:40,end-40:end])=0; %get rid of end effects...

pp = medianFilter(pitch_v',10);         % rad
pv = meanFilter(diff(pp),40)./viconPer; % rad/s
pa = meanFilter(diff(pv),40)./viconPer; % rad/s^2
pv = [0; pv];
pa = [0; 0; pa];
pa([1:40,end-40:end])=0; %get rid of end effects...

yp = medianFilter(yaw_v',10);           % rad
yv = meanFilter(diff(yp),40)./viconPer; % rad/s
ya = meanFilter(diff(yv),40)./viconPer; % rad/s^2
yv = [0; yv];
ya = [0; 0; ya];
ya([1:40,end-40:end])=0; %get rid of end effects...

% Vicon rotational velocity (rad/s)
p = double(ones(size(rv,1),1).*rv) - double(yv.*sin(pp));
q = double(pv.*cos(rp)) + double(yv.*sin(rp).*cos(pp));
r = double(-pv.*sin(rp)) + double(yv.*cos(rp).*cos(pp));

% Estimate desired yaw acc
ch4_d = meanFilter(diff(ch4)',10)./apmPer;
ch4_d = [0; ch4_d];

%% Motor Parameter Estimation

% Compute Motor Speed Command Inputs
Quad_KT = 1.3328e-5;   % Thrust force coeffecient (kg-m)
Quad_Kd = 1.3858e-6;  % Drag torque coeffecient (kg-m^2)
Quad_l = 0.56;       % Distance from the center of mass to the each motor (m)

% Compute Motor Speed from ESC Ouput
RCPer = median(diff(pwm_time)); %Sampling frequency
tau = 0.15; % Motor time constant (s)
DC = 6.5;   % Motor DC gain
s = tf('s');
G = DC/(tau*s+1);

sysD = c2d(G,RCPer,'zoh');

for(k = 1:size(t_pwm_out,2))
    if k == 1
        RPM_1(1,k) = 0;
        RPM_2(1,k) = 0;
        RPM_3(1,k) = 0;
        RPM_4(1,k) = 0;
    end
    if k > 1
        RPM_1(1,k) = .4936*RPM_1(1,k-1) + 3.292*pwm_out(3,k-1);
        RPM_2(1,k) = .4936*RPM_2(1,k-1) + 3.292*pwm_out(1,k-1);
        RPM_3(1,k) = .4936*RPM_3(1,k-1) + 3.292*pwm_out(4,k-1);
        RPM_4(1,k) = .4936*RPM_4(1,k-1) + 3.292*pwm_out(2,k-1);
    end
end

% Convert to rad/s
RPM_1 = (RPM_1-5525)*(2*pi/60);
RPM_2 = (RPM_2-5525)*(2*pi/60);
RPM_3 = (RPM_3-5525)*(2*pi/60);
RPM_4 = (RPM_4-5525)*(2*pi/60);

tinds = t_pwm_out >= t_imu(t_start) & t_pwm_out <= t_imu(t_end);
RPM_1 = RPM_1(tinds);
RPM_2 = RPM_2(tinds);
RPM_3 = RPM_3(tinds);
RPM_4 = RPM_4(tinds);

% Compares motor speed (rad/s) calculated with Kv vs motor model
% figure();
% subplot(4,1,1);
% plot(pwm_time, m1,pwm_time, RPM_1);
% subplot(4,1,2);
% plot(pwm_time, m2,pwm_time, RPM_2);
% subplot(4,1,3);
% plot(pwm_time, m3,pwm_time, RPM_3);
% subplot(4,1,4);
% plot(pwm_time, m4,pwm_time, RPM_4);
% legend('Kv','Model');

% Compute model control inputs using Kv
U1 = Quad_KT*(m1.^2+m2.^2+m3.^2+m4.^2);     % Thrust (kg-m/s^2)
U2 = Quad_KT*Quad_l*(m4.^2-m2.^2);          % Roll   (kg-m/s^2)
U3 = Quad_KT*Quad_l*(m3.^2-m1.^2);          % Pitch  (kg-m/s^2)
U4 = Quad_Kd*(m1.^2-m2.^2+m3.^2-m4.^2);     % Yaw    (kg-m^2/s^2)
Om = m1-m2+m3-m4;                           % rad/s

% Compute model control inputs using motor model
U1 = Quad_KT*(RPM_1.^2+RPM_2.^2+RPM_3.^2+RPM_4.^2);     % Thrust (kg-m/s^2)
U2 = Quad_KT*Quad_l*(RPM_4.^2-RPM_2.^2);                % Roll   (kg-m/s^2)
U3 = Quad_KT*Quad_l*(RPM_3.^2-RPM_1.^2);                % Pitch  (kg-m/s^2)
U4 = Quad_Kd*(RPM_1.^2-RPM_2.^2+RPM_3.^2-RPM_4.^2);     % Yaw    (kg-m^2/s^2)
Om = RPM_1-RPM_2+RPM_3-RPM_4;                           % rad/s

U2 = meanFilter(U2',2)';
U3 = meanFilter(U3',2)';

%% Compare values

% temp = meanFilter(gyro_r',2);
% plotyy(vicon_time,pa,pwm_time,-U3);
% legend('Y pos','U2','roll','RC');
% 
% figure()
% plotyy(vicon_time,ra,pwm_time,U2);
% legend('Y pos','U2','roll','RC');
% 
% figure()
% plotyy(vicon_time,ya,pwm_time,-U4);
% legend('Y pos','U2','roll','RC');
% break;

%% Find Delays
% Search for best alignment of vicon and RC control data for each control
% channel. 
offsets = linspace(-.5,.5,50);

RMSp = zeros(size(offsets));
RMSr = zeros(size(offsets));
RMSz = zeros(size(offsets));
RMSy = zeros(size(offsets));

i = 0;
for o = offsets
    i = i+1;
    vinds = findLatestInds(rc_time,vicon_time+o);
    
    ccf = corrcoef(double([pp(vinds),ch2']));
    RMSp(i) = ccf(1,2);
    
    ccf = corrcoef(double([rp(vinds),ch1']));
    RMSr(i) = ccf(1,2);
    
    ccf = corrcoef(double([-gza(vinds),ch3']));
    RMSz(i) = ccf(1,2);
    
    ccf = corrcoef(double([yv(vinds),ch4']));
    RMSy(i) = ccf(1,2);
        
end

disp(' ');
[v oi] = max(RMSp);
vindsp = findLatestInds(rc_time,vicon_time+offsets(oi));
disp(['Vicon/RC pitch angle delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMSr);
vindsr = findLatestInds(rc_time,vicon_time+offsets(oi));
disp(['Vicon/RC roll angle delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMSz);
vindsz = findLatestInds(rc_time,vicon_time+offsets(oi));
disp(['Vicon/RC thrust delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMSy);
vindsy = findLatestInds(rc_time,vicon_time+offsets(oi));
disp(['Vicon/RC yaw rate delay = ' num2str(offsets(oi)) 'sec']);
disp(' ');

if(1)
    
    figure(35)
    subplot(4,1,1);
    plot(rc_time,pp(vindsp),rc_time,ch2')
    title('Vicon Pitch vs. RC Pitch Command')
    legend('Measured Pitch','Desired Pitch');
    ylabel('Angle (rad)');
    xlabel('Time (s)');
    
    subplot(4,1,2);
    plot(rc_time,rp(vindsr),rc_time,ch1')
    title('Vicon Roll vs. RC Roll Command')
    legend('Measured Roll','Desired Roll');
    ylabel('Angle (rad)');
    xlabel('Time (s)');
    
    subplot(4,1,3);
    plot(rc_time,-gza(vindsz),rc_time,ch3')
    title('Vicon Z Acc vs. RC Thrust Command')
    legend('Measured Z acc','Desired Thurst');
    ylabel('Acceleration (m/s^2)');
    xlabel('Time (s)');
    
    subplot(4,1,4);
    plot(rc_time,yv(vindsy),rc_time,ch4')
    title('Vicon Yaw Vel vs. RC Yaw Command')
    legend('Measured Yaw Rate','Desired Yaw Rate');
    ylabel('Angular Velocity (rad/s)');
    xlabel('Time (s)');
    
end

% Search for best alignment of vicon and motor control data for each control
% channel.

offsets = linspace(0,.5,100);

RMSp = zeros(size(offsets));
RMSr = zeros(size(offsets));
RMSz = zeros(size(offsets));
RMSy = zeros(size(offsets));

i = 0;
for o = offsets
    i = i+1;
    vinds = findLatestInds(pwm_time,vicon_time+o);
    
    ccf = corrcoef(double([-pa(vinds),U3']));   %neg for w3-w1
    RMSp(i) = ccf(1,2);
    
    ccf = corrcoef(double([ra(vinds),U2']));
    RMSr(i) = ccf(1,2);
    
    ccf = corrcoef(double([-gza(vinds),U1']));
    RMSz(i) = ccf(1,2);
    
    ccf = corrcoef(double([-ya(vinds),U4']));
    RMSy(i) = ccf(1,2);
        
end

disp(' ');
[v oi] = max(RMSp);
vindsU3 = findLatestInds(pwm_time,vicon_time+offsets(oi));
disp(['Vicon/ESC pitch acc delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMSr);
vindsU2 = findLatestInds(pwm_time,vicon_time+offsets(oi));
disp(['Vicon/ESC roll acc delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMSz);
vindsU1 = findLatestInds(pwm_time,vicon_time+offsets(oi));
disp(['Vicon/ESC z acc delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMSy);
vindsU4 = findLatestInds(pwm_time,vicon_time+offsets(oi));
disp(['Vicon/ESC yaw acc delay = ' num2str(offsets(oi)) 'sec']);
disp(' ');

if(0)
    
    figure(5)
    subplot(4,1,1);
    plotyy(pwm_time,-pa(vindsU3),pwm_time,U3')
    title('Vicon Pitch Acceleration vs. Pitch Control Input')
    legend('Measured Acceleration','Control Input');
    ylabel('Acceleration (rad/s^2)');
    xlabel('Time (s)');
    
    subplot(4,1,2);
    plotyy(pwm_time,ra(vindsU2),pwm_time,U2')
    title('Vicon Roll Acceleration vs. Roll Control Input')
    legend('Measured Acceleration','Control Input');
    ylabel('Acceleration (rad/s^2)');
    xlabel('Time (s)');
    
    subplot(4,1,3);
    plotyy(pwm_time,-gza(vindsU1),pwm_time,U1')
    title('Vicon Z Axis Acceleration vs. Thrust Control Input')
    legend('Measured Acceleration','Control Input');
    ylabel('Acceleration (m/s^2)');
    xlabel('Time (s)');
    
    subplot(4,1,4);
    plotyy(pwm_time,-ya(vindsU4),pwm_time,U4')
    title('Vicon Yaw Acceleration vs. Yaw Control Input')
    legend('Measured Acceleration','Control Input');
    ylabel('Acceleration (rad/s^2)');
    xlabel('Time (s)');
    
end

%% RC in (deg, thurst, yaw rate) and ESC Out (rad/s) delay 
offsets = linspace(-1.5,1.5,100);

RMS1 = zeros(size(offsets));
RMS2 = zeros(size(offsets));
RMS3 = zeros(size(offsets));
RMS4 = zeros(size(offsets));

i = 0;
for o = offsets
    i = i+1;
    vinds = findLatestInds(rc_time,pwm_time+o);
    
    ccf = corrcoef(double([U1(vinds)',ch3']));
    RMS1(i) = ccf(1,2);
    
    ccf = corrcoef(double([U2(vinds)',ch1']));
    RMS2(i) = ccf(1,2);
    
    ccf = corrcoef(double([U3(vinds)',ch2']));
    RMS3(i) = ccf(1,2);
    
    ccf = corrcoef(double([U4(vinds)',ch4']));
    RMS4(i) = ccf(1,2);
           
end

disp(' ');
[v oi] = max(RMS1);

vinds1 = findLatestInds(rc_time,pwm_time+offsets(oi));
disp(['RC/ESC Thrust delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMS2);

vinds2 = findLatestInds(rc_time,pwm_time+offsets(oi));
disp(['RC/ESC Roll delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMS3);

vinds3 = findLatestInds(rc_time,pwm_time+offsets(oi));
disp(['RC/ESC Pitch delay = ' num2str(offsets(oi)) 'sec']);

[v oi] = max(RMS4);

vinds4 = findLatestInds(rc_time,pwm_time+offsets(oi));
disp(['RC/ESC Yaw delay = ' num2str(offsets(oi)) 'sec']);
disp(' ');

if(0)
    
    figure(34)
    title('Delay Compensation Only');
    subplot(4,1,1);
    plotyy(rc_time,U1(vinds1)',rc_time,ch3')
    title('Motor Thrust Command vs. RC Thrust Command')
    legend('ESC','RC');
    
    subplot(4,1,2);
    plotyy(rc_time,U2(vinds2)',rc_time,ch1')
    title('Motor Roll Torque Command vs. RC Roll Command')
    legend('ESC','RC');
    
    subplot(4,1,3);
    plotyy(rc_time,U3(vinds3)',rc_time,ch2')
    title('Motor Pitch Torque Command vs. RC Pitch Command')
    legend('ESC','RC');
    
    subplot(4,1,4);
    plotyy(rc_time,U4(vinds4)',rc_time,ch4')
    title('Motor Yaw Torque Command vs. RC Yaw Command')
    legend('ESC','RC');
    
end
%% Least Squares Fit For Model Verification

Quad_m = 1.4;      % Quadrotor mass (kg)
Quad_g = 9.8;      % Gravity (m/s^2)

Quad_Kdx = .16481;    % Translational drag force coeffecient (kg/s)
Quad_Kdy = .31892;    % Translational drag force coeffecient (kg/s)
Quad_Kdz = .001;    % Translational drag force coeffecient (kg/s)

Quad_Jx = .05;     % Moment of inertia about X axis (kg-m^2)
Quad_Jy = .05;     % Moment of inertia about Y axis (kg-m^2)
Quad_Jz = .24;     % Moment of inertia about Z axis (kg-m^2)
Quad_Jp = .044;    % Moment of Intertia of the rotor (kg-m^2)

% Complex Global Frame Translation Model
if(1)   
    
    % x_accel = m1*Rx*u1 - m2*Kdx*Xd + b    
    Rx = (1/Quad_m)*U1'.*(cos(rp(vindsU2)).*cos(yp(vindsU4)).*sin(pp(vindsU3)) + sin(rp(vindsU2)).*sin(yp(vindsU4)));
    Ax = [-Rx, -(1/Quad_m)*Quad_Kdx*gxv(vindsU3), ones(size(Rx,1),1)];
    Bx = gxa(vindsU3);
    [kxn,loosxn,bestlambdasxn] = lrlsloobest(Ax, Bx);
    
    % y_accel = m1*Ry*u1 - m2*Kdy*Yd + b
    Ry = (1/Quad_m)*U1'.*(cos(rp(vindsU2)).*sin(yp(vindsU4)).*sin(pp(vindsU3)) - sin(rp(vindsU2)).*cos(yp(vindsU4)));
    Ay = [-Ry, -(1/Quad_m)*Quad_Kdy*gyv(vindsU2), ones(size(Ry,1),1)];
    By = gya(vindsU2);
    [kyn,loosyn,bestlambdasyn] = lrlsloobest(Ay, By);
    
    % z_accel = m1*Ry*u1 - m2*Kdz*Xd + g*b
    Rz = (1/Quad_m)*U1'.*(cos(rp(vindsU2)).*cos(pp(vindsU3)));
    Az = [-Rz, -(1/Quad_m)*Quad_Kdz*gzv(vindsU1), Quad_g*ones(size(Rz,1),1)];
    Bz = gza(vindsU1);
    [kzn,looszn,bestlambdaszn] = lrlsloobest(Az, Bz);
               
    figure(36)
    
    subplot(3,1,1);
    plot(pwm_time,[gxa(vindsU3),Ax*kxn])
    legend('Measured','Model')
    rmse_x = sqrt( sum( (gxa(vindsU3)-Ax*kxn).^2) /numel(gxa(vindsU3)) );
    str = sprintf('X Axis Acceleration (gxa = m1*Rx*u1 - m2*Kdx*Xd + b) RMSE = %.3f m/s^2',rmse_x);
    title(str);
    xlabel('Time (sec)');
    ylabel('Acceleration (m/s^2)');
    
    subplot(3,1,2);
    plot(pwm_time,[gya(vindsU2),Ay*kyn])
    legend('Measured','Model')
    rmse_y = sqrt( sum( (gya(vindsU2)-Ay*kyn).^2) /numel(gya(vindsU2)) );
    str = sprintf('Y Axis Acceleration (gya = m1*Ry*u1 - m2*Kdy*Yd + b) RMSE = %.3f m/s^2',rmse_y);
    title(str)
    xlabel('Time (sec)');
    ylabel('Acceleration (m/s^2)');
    
    subplot(3,1,3);
    plot(pwm_time,[gza(vindsU1),Az*kzn])
    legend('Measured','Model')
    rmse_z = sqrt( sum( (gza(vindsU1)-Az*kzn).^2) /numel(gza(vindsU1)) );
    str = sprintf('Z Axis Acceleration (gza = m1*Rz*u1 - m2*Kdz*Zd + g*b) RMSE = %.3f m/s^2',rmse_z);
    title(str)
    xlabel('Time (sec)');
    ylabel('Acceleration (m/s^2)');
    
    disp(['KT = [' num2str(Quad_KT*mean([kxn(1,1),kyn(1,1),kzn(1,1)])) '];']);
    disp(['Kdx = [' num2str(Quad_Kdx*kxn(2,1)) '];']);
    disp(['Kdy = [' num2str(Quad_Kdy*kyn(2,1)) '];']);
    disp(['Kdz = [' num2str(Quad_Kdz*kzn(2,1)) '];']);
end

% Complex Global Frame Rotation Model
if(1)
    
    U1 = U1(vinds1); 
    U2 = U2(vinds2); 
    U3 = U3(vinds3); 
    U4 = U4(1:end-1); 
    Om = Om(vinds4);
          
    % rdd = m1*Jwx*q*r - m2*Jrx*q*Om + m3*u2 + b    
    Jwx = ((Quad_Jy-Quad_Jz)/Quad_Jx)*q(vindsp).*r(vindsy);
    Jrx = (Quad_Jp*q(vindsp).*Om'/Quad_Jx);
    Ar = [U2', ones(size(vindsr,2),1)];
    Ar2 = [Jwx, -Jrx, Quad_l*(U2./Quad_Jx)', ones(size(vindsr,2),1)];    
    Ar3 = [ch1', ones(size(ch1,2),1)];
    Br = ra(vindsr);
    [krn,loosrn,bestlambdasrn] = lrlsloobest(Ar, Br);
    [krn2,loosrn2,bestlambdasrn2] = lrlsloobest(Ar2, Br);
    [krn3,loosrn3,bestlambdasrn3] = lrlsloobest(Ar3, Br);
    
    % pdd = m1*Jwy*p*r - m2*Jry*p*Om + m3*u3 + b    
    Jwy = ((Quad_Jz-Quad_Jx)/Quad_Jy)*p(vindsr).*r(vindsy);
    Jry = (Quad_Jp*p(vindsr)/Quad_Jy);    
    Ap = [U3', ones(size(vindsp,2),1)];   
    Ap2 = [Jwy, Jry, Quad_l*(U3./Quad_Jy)', ones(size(vindsp,2),1)];
    Ap3 = [ch2', ones(size(ch2,2),1)];
    Bp = pa(vindsp);
    [kpn,loospn,bestlambdaspn] = lrlsloobest(Ap, Bp);
    [kpn2,loospn2,bestlambdaspn2] = lrlsloobest(Ap2, Bp);
    [kpn3,loospn3,bestlambdaspn3] = lrlsloobest(Ap3, Bp);
    
    % hdd = m1*Jwz*p*q + m2*u4 + b    
    Jwz = ((Quad_Jx-Quad_Jy)/Quad_Jz)*p(vindsr).*q(vindsp);    
    Ah = [U4', ones(size(vindsy,2),1)];
    Ah2 = [Jwz, (U4./Quad_Jz)', ones(size(vindsy,2),1)];
    Ah3 = [ch4', ones(size(ch4,2),1)];
    Bh = ya(vindsU4(1:end-1));
    [khn,looshn,bestlambdashn] = lrlsloobest(Ah, Bh);
    [khn2,looshn2,bestlambdashn2] = lrlsloobest(Ah2, Bh);
    [khn3,looshn3,bestlambdashn3] = lrlsloobest(Ah3, Bh);       

    disp(['Jp = [' num2str(Quad_Jp*mean([abs(krn2(2,1)),abs(kpn2(2,1))])) '];']);
    disp(['KT = [' num2str(Quad_KT*mean([abs(krn2(3,1)),abs(kpn2(3,1))])) '];']);
    disp(['Kd = [' num2str(Quad_Kd*khn2(2,1)) '];']);
         
    figure(370)
    
    subplot(3,1,1);
    plot(rc_time,[ra(vindsr),Ar2*krn2])
    legend('Measured','Model','Complex','RC');
    rmse_r = sqrt( sum( (ra(vindsr)-Ar*krn).^2) /numel(ra(vindsr)) );
    rmse_r2 = sqrt( sum( (ra(vindsr)-Ar2*krn2).^2) /numel(ra(vindsr)) );
    rmse_r = sqrt( sum( (ra(vindsr)-Ar3*krn3).^2) /numel(ra(vindsr)) );
    str = sprintf('Roll Angular Acceleration (rdd = m1*Jwx*q*r - m2*Jrx*q*Om + m3*u2 + b) RMSE = %.3f rad/s^2',rmse_r2);
    title(str);
    xlabel('Time (sec)');
    ylabel('Acceleration (rad/s^2)');
    
    subplot(3,1,2);
    plot(rc_time,[pa(vindsp),Ap2*kpn2])
    legend('Measured','Model','Complex','RC');
    rmse_p = sqrt( sum( (pa(vindsp)-Ap*kpn).^2) /numel(pa(vindsp)) );
    rmse_p2 = sqrt( sum( (pa(vindsp)-Ap2*kpn2).^2) /numel(pa(vindsp)) );
    rmse_p = sqrt( sum( (pa(vindsp)-Ap3*kpn3).^2) /numel(pa(vindsp)) );
    str = sprintf('Pitch Angular Acceleration (pdd = m1*Jwy*p*r - m2*Jry*p*Om + m3*u3 + b) RMSE = %.3f rad/s^2',rmse_p2);
    title(str)
    xlabel('Time (sec)');
    ylabel('Acceleration (rad/s^2)');
    
    subplot(3,1,3);
    plot(rc_time,[ya(vindsU4(1:end-1)),Ah2*khn2])
    legend('Measured','Model','Complex','RC');
    rmse_h = sqrt( sum( (ya(vindsy)-Ah*khn).^2) /numel(ya(vindsy)) );
    rmse_h2 = sqrt( sum( (ya(vindsU4(1:end-1))-Ah2*khn2).^2) /numel(ya(vindsy)) );
    rmse_h = sqrt( sum( (ya(vindsy)-Ah3*khn3).^2) /numel(ya(vindsy)) );
    str = sprintf('Yaw Angular Acceleration (ydd = m1*Jwz*p*q + m2*u4 + b) RMSE = %.3f rad/s^2',rmse_h2);
    title(str)
    xlabel('Time (sec)');
    ylabel('Acceleration (rad/s^2)');
    
end

% Simple Body Frame Model Using Angle and Motor Inputs
if(0)
        
    % x_accel = m*pitch*U1 + b    (kx = [m, b])
%     Ax=[-(1/Quad_m)*pp(vindsp).*U1',ones(size(ch3,2),1)];
    Ax=[-(1/Quad_m)*pp(vindsp).*U1',ones(size(U1,2),1)];
    Bx = bxa(vindsp);
    [kx,loosx,bestlambdasx] = lrlsloobest(Ax, Bx);
           
    % y_accel = m*roll*U1 + b    (ky = [m, b])
    Ay=[(1/Quad_m)*rp(vindsr).*U1',ones(size(U1,2),1)];
    By = bya(vindsr);
    [ky,loosy,bestlambdasy] = lrlsloobest(Ax, Bx);
    
    % z_accel = m*U1 + b    (kz = [m, b])
    Az=[-(1/Quad_m)*U1',Quad_g*ones(size(U1,2),1)];
    Bz = gza(vindsz);
    [kz,loosz,bestlambdasz] = lrlsloobest(Az, Bz);
    
    % yaw_vel = m * yaw + b    (kh = [m, b])
%     Ah=[ch4',ones(size(ch4,2),1)];
%     Bh = yv(vindsy);
%     [kh,loost,bestlambdast] = lrlsloobest(Ah, Bh);
    
    
%     figure(350)
%     
%     subplot(3,1,1);
%     plot(Ax(:,1), Bx(:), '.'); hold on;
%     x_x = [min(Ax(:,1))-1:.01:max(Ax(:,1))+1];
%     y_x = kx(1)*x_x + kx(2);
%     plot(x_x, y_x);
%     
%     subplot(3,1,2);
%     plot(Ay(:,1), By(:), '.'); hold on;
%     x_y = [min(Ay(:,1))-1:.01:max(Ay(:,1))+1];
%     y_y = ky(1)*x_y + ky(2);
%     plot(x_y, y_y);
%     
%     subplot(3,1,3);
%     plot(Az(:,1), Bz(:), '.'); hold on;
%     x_z = [min(Az(:,1))-1:.01:max(Az(:,1))+1];
%     y_z = kz(1)*x_z + kz(2);
%     plot(x_z, y_z);

    figure(380)
    
    subplot(3,1,1);
    plot(rc_time,[bxa(vindsp),Ax*kx])
    rmse_x = sqrt( sum( (bxa(vindsp)-Ax*kx).^2) /numel(bxa(vindsp)) );
    legend('Measured','Model')
    str = sprintf('gxa = m1*pitch*u1 + b RMSE = %.3f mm/s^2',rmse_x);
    title(str)
    
    subplot(3,1,2);
    plot(rc_time,[bya(vindsr),Ay*ky])
    rmse_y = sqrt( sum( (bya(vindsr)-Ay*ky).^2) /numel(bya(vindsp)) );
    legend('Measured','Model')
    str = sprintf('gya = m1*roll*u1 + b RMSE = %.3f mm/s^2',rmse_y);
    title(str)
    
    subplot(3,1,3);
    plot(rc_time,[gza(vindsz),Az*kz]);
    rmse_z = sqrt( sum( (gza(vindsz)-Az*kz).^2) /numel(gza(vindsz)) );
    legend('Measured','Model')
    str = sprintf('gza = m1*u1 + g*b RMSE = %.3f mm/s^2',rmse_z);
    title(str)
    
end

% Simple Body Frame Model Using RC Inputs
if(0)

    % x_accel = m * pitch + b    (kx = [m, b])
    Ax=[ch2',ones(size(ch2,2),1)];
    Ax2=[ch2',bxv(vindsp),ones(size(ch2,2),1)];
    Bx = bxa(vindsp);
    [kx,loosx,bestlambdasx] = lrlsloobest(Ax, Bx);
    [kx2,loosx2,bestlambdasx2] = lrlsloobest(Ax2, Bx);
    
    % y_accel = m * roll + b    (ky = [m, b])
    Ay=[-ch1',ones(size(ch1,2),1)];
    Ay2=[-ch1',byv(vindsr),ones(size(ch1,2),1)];
    By = bya(vindsr);
    [ky,loosy,bestlambdasy] = lrlsloobest(Ax, Bx);
    [ky2,loosy2,bestlambdasy2] = lrlsloobest(Ay2, By);
    
    % z_accel = m * thrust + b    (kz = [m, b])
    Az=[ch3',ones(size(ch3,2),1)];
    Bz = gza(vindsz);
    [kz,loosz,bestlambdasz] = lrlsloobest(Az, Bz);
    
    % yaw_vel = m * yaw + b    (kh = [m, b])
    Ah=[ch4',ones(size(ch4,2),1)];
    Bh = yv(vindsy);
    [kh,loost,bestlambdast] = lrlsloobest(Ah, Bh);
    
    
%     figure(350)
%     
%     subplot(4,1,1);
%     plot(Ax(:,1), Bx(:), '.'); hold on;
%     x_x = [min(Ax(:,1))-1:.01:max(Ax(:,1))+1];
%     y_x = kx(1)*x_x + kx(2);
%     plot(x_x, y_x);
%     
%     subplot(4,1,2);
%     plot(Ay(:,1), By(:), '.'); hold on;
%     x_y = [min(Ay(:,1))-1:.01:max(Ay(:,1))+1];
%     y_y = ky(1)*x_y + ky(2);
%     plot(x_y, y_y);
%     
%     subplot(4,1,3);
%     plot(Az(:,1), Bz(:), '.'); hold on;
%     x_z = [min(Az(:,1))-1:.01:max(Az(:,1))+1];
%     y_z = kz(1)*x_z + kz(2);
%     plot(x_z, y_z);
%     
%     subplot(4,1,4);
%     plot(Ah(:,1), Bh(:), '.'); hold on;
%     x_t = [min(Ah(:,1))-1:.01:max(Ah(:,1))+1];
%     y_t = kh(1)*x_t + kh(2);
%     plot(x_t, y_t);


    figure(360)
    
    subplot(4,1,1);
    plot(rc_time,[bxa(vindsp),Ax*kx,Ax2*kx2])
    rmse_x = sqrt( sum( (bxa(vindsp)-Ax*kx).^2) /numel(bxa) );
    str = sprintf('aligned vicon acc vs eservo*Kx RMSE = %.3f mm/s^2',rmse_x);
    title(str)
    
    subplot(4,1,2);
    plot(rc_time,[bya(vindsr),Ay*ky,Ay2*ky2])
    rmse_y = sqrt( sum( (bya(vindsr)-Ay*ky).^2) /numel(bya) );
    str = sprintf('aligned vicon acc vs eservo*Ky RMSE = %.3f mm/s^2',rmse_y);
    title(str)
    
    subplot(4,1,3);
    plot(rc_time,[gza(vindsz),Az*kz])
    title('aligned z acc vs. eservo*Kz')
    
    subplot(4,1,4);
    plot(rc_time,[yv(vindsy),Ah*kh])
    title('aligned yaw vel vs. eservo*Kt')
    
end

% Complex Global Frame Rotation Model #2
if(0)
         
    % rdd = m1*Jwx*q*r - m2*Jrx*q*Om + m3*u2 + b    
    Jwx = ((Quad_Jy-Quad_Jz)/Quad_Jx)*q(vindsU3).*r(vindsU4);
    Jrx = (Quad_Jp*q(vindsU3).*Om'/Quad_Jx);
    Ar = [U2', ones(size(U2,2),1)];
    Ar2 = [Jwx, -Jrx, Quad_l*(U2./Quad_Jx)', ones(size(U2,2),1)];    
%     Ar3 = [ch1', ones(size(ch1,2),1)];
    Br = ra(vindsU2);
    [krn,loosrn,bestlambdasrn] = lrlsloobest(Ar, Br);
    [krn2,loosrn2,bestlambdasrn2] = lrlsloobest(Ar2, Br);
%     [krn3,loosrn3,bestlambdasrn3] = lrlsloobest(Ar3, Br);
    
    % pdd = m1*Jwy*p*r + m2*Jry*p*Om + m3*u3 + b    
    Jwy = ((Quad_Jz-Quad_Jx)/Quad_Jy)*p(vindsU2).*r(vindsU4);
    Jry = (Quad_Jp*p(vindsU2).*Om'/Quad_Jy);    
    Ap = [U3', ones(size(U3,2),1)];   
    Ap2 = [Jwy, Jry, Quad_l*(U3./Quad_Jy)', ones(size(U3,2),1)];
%     Ap3 = [ch2', ones(size(ch2,2),1)];
    Bp = pa(vindsU3);  %neg is w3-w1
    [kpn,loospn,bestlambdaspn] = lrlsloobest(Ap, Bp);
    [kpn2,loospn2,bestlambdaspn2] = lrlsloobest(Ap2, Bp);
%     [kpn3,loospn3,bestlambdaspn3] = lrlsloobest(Ap3, Bp);
    
    % hdd = m1*Jwz*p*q + m2*u4 + b    
    Jwz = ((Quad_Jx-Quad_Jy)/Quad_Jz)*p(vindsU2).*q(vindsU3);    
    Ah = [U4', ones(size(U4,2),1)];
    Ah2 = [Jwz, (U4./Quad_Jz)', ones(size(U4,2),1)];
%     Ah3 = [ch4', ones(size(ch4,2),1)];
    Bh = ya(vindsU4);  %neg if using model motor order
    [khn,looshn,bestlambdashn] = lrlsloobest(Ah, Bh);
    [khn2,looshn2,bestlambdashn2] = lrlsloobest(Ah2, Bh);
%     [khn3,looshn3,bestlambdashn3] = lrlsloobest(Ah3, Bh);
    
    disp(['Jp = [' num2str(Quad_Jp*mean([krn2(2,1),kpn2(2,1)])) '];']);
    disp(['KT = [' num2str(Quad_KT*mean([krn2(3,1),kpn2(3,1)])) '];']);
    disp(['Kd = [' num2str(Quad_Kd*khn2(2,1)) '];']);
      
    figure(37)
    
    subplot(3,1,1);
    plot(pwm_time,[ra(vindsU2),Ar2*krn2,U2'])
    legend('Measured','Complex','Simple');
    rmse_r = sqrt( sum( (ra(vindsU2)-Ar*krn).^2) /numel(ra(vindsU2)) );
    rmse_r2 = sqrt( sum( (ra(vindsU2)-Ar2*krn2).^2) /numel(ra(vindsU2)) );
%     rmse_r = sqrt( sum( (ra(vindsU2)-Ar3*krn3).^2) /numel(ra(vindsU2)) );
    str = sprintf('rdd = m1*Jwx*q*r - m2*Jrx*q*Om + m3*u2 + b RMSE = %.3f rad/s^2',rmse_r2);
    title(str);
    xlabel('Time (sec)');
    ylabel('Acceleration (rad/s^2)');
    
    subplot(3,1,2);
    plot(pwm_time,[pa(vindsU3),Ap2*kpn2,-U3'])
    legend('Measured','Complex','Simple');
    rmse_p = sqrt( sum( (pa(vindsU3)-Ap*kpn).^2) /numel(pa(vindsU3)) );
    rmse_p2 = sqrt( sum( (pa(vindsU3)-Ap2*kpn2).^2) /numel(pa(vindsU3)) );
%     rmse_p = sqrt( sum( (pa(vindsU3)-Ap3*kpn3).^2) /numel(pa(vindsU3)) );
    str = sprintf('pdd = m1*Jwy*p*r - m2*Jry*p*Om + m3*u3 + b RMSE = %.3f rad/s^2',rmse_p2);
    title(str)
    xlabel('Time (sec)');
    ylabel('Acceleration (rad/s^2)');
    
    subplot(3,1,3);
    plot(pwm_time,[ya(vindsU4),Ah2*khn2,Ah*khn])
    legend('Measured','Complex','Simple');
    rmse_h = sqrt( sum( (ya(vindsU4)-Ah*khn).^2) /numel(ya(vindsU4)) );
    rmse_h2 = sqrt( sum( (ya(vindsU4)-Ah2*khn2).^2) /numel(ya(vindsU4)) );
%     rmse_h = sqrt( sum( (ya(vindsU4)-Ah3*khn3).^2) /numel(ya(vindsU4)) );
    str = sprintf('hdd = m1*Jwz*p*q + m2*u4 + b RMSE = %.3f rad/s^2',rmse_h2);
    title(str)
    xlabel('Time (sec)');
    ylabel('Acceleration (rad/s^2)');
    
end

