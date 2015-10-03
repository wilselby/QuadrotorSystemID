% Wil Selby
% Washington, DC
% July 08, 2015

% This script processes .mat files comprised of ArduPilot Mavlink messages
% converted from .tlog files to .mat files in Mission Planner. This script
% will model the accelermoter, gyroscope, and GPS sensors

%% Initialization

clear all;
close all;
clc;

% Add Paths
addpath logs
addpath utilities
addpath 'C:\Users\test\Dropbox\UAS Research\MATLAB\SysID\utilities\Smooth'
addpath 'C:\Users\test\Dropbox\UAS Research\MATLAB\SysID\utilities\angleStuff'
addpath 'C:\Users\test\Dropbox\UAS Research\MATLAB\SysID\utilities\rls\matlab'

%% Load Log Data
load IMUlog100Hz2;   

%% Formatting for plots

FontName = 'Arial';
FontSize = 14;
plotlinewidth=2;

%% Extract Data

t_start = 1;
t_end = size(IMUlog100Hz2,1);

% Time
time = IMUlog100Hz2(t_start:t_end,1)/1000;
time = time-time(1);
IMUPer = median(diff(time));

% Acceleromter (m/s^2)
x_acc = IMUlog100Hz2(t_start:t_end,2);
x_acc_bias = mean(x_acc);
x_acc_nobias = x_acc - x_acc_bias;
x_acc_mdl = x_acc_bias + std(x_acc).*randn(size(time,1),1);
DATA = struct('freq',x_acc,'rate',100);
% DATA = struct('freq',x_acc,'time',time);
[adxg,empty,empty,tauxg] = allan(DATA,'','data',0);

y_acc = IMUlog100Hz2(t_start:t_end,3);
y_acc_bias = mean(y_acc);
y_acc_nobias = y_acc - y_acc_bias;
y_acc_mdl = y_acc_bias + std(y_acc).*randn(size(time,1),1);
DATA = struct('freq',y_acc,'rate',100);
% DATA = struct('freq',x_acc,'time',time);
[adyg,empty,empty,tauyg] = allan(DATA,'','data',0);

z_acc = IMUlog100Hz2(t_start:t_end,4);
z_acc_bias = mean(z_acc);
z_acc_nobias = z_acc - z_acc_bias;
z_acc_mdl = z_acc_bias + std(z_acc).*randn(size(time,1),1);
DATA = struct('freq',z_acc,'rate',100);
% DATA = struct('freq',x_acc,'time',time);
[adzg,empty,empty,tauzg] = allan(DATA,'','data',0);

% Gyroscope (deg/s)
x_gyro = IMUlog100Hz2(t_start:t_end,6);
x_gyro_bias = mean(x_gyro);
x_gyro_nobias = x_gyro - x_gyro_bias;
x_gyro_mdl = x_gyro_bias + std(x_gyro).*randn(size(time,1),1);
DATA = struct('freq',x_gyro,'rate',100);
% DATA = struct('freq',x_acc,'time',time);
[adx,empty,empty,taux] = allan(DATA,'','data',0);

y_gyro = IMUlog100Hz2(t_start:t_end,7);
y_gyro_bias = mean(y_gyro);
y_gyro_nobias = y_gyro - y_gyro_bias;
y_gyro_mdl = y_gyro_bias + std(y_gyro).*randn(size(time,1),1);
DATA = struct('freq',y_gyro,'rate',100);
% DATA = struct('freq',x_acc,'time',time);
[ady,empty,empty,tauy] = allan(DATA,'','data',0);

z_gyro = IMUlog100Hz2(t_start:t_end,8);
z_gyro_bias = mean(z_gyro);
z_gyro_nobias = z_gyro - z_gyro_bias;
z_gyro_mdl = z_gyro_bias + std(z_gyro).*randn(size(time,1),1);
DATA = struct('freq',z_gyro,'rate',100);
% DATA = struct('freq',x_acc,'time',time);
[adz,empty,empty,tauz] = allan(DATA,'','data',0);

%Allan test

for(i = 1: size(x_gyro,1))
    n(i,1) = (6.468e-5/sqrt(.01))*randn(1);
    if(i>1)
        b(i,1) = b(i-1,1) + 3.818e-5*sqrt(.01)*randn(1);
    else
        b(i,1) = 1.5e-5*sqrt(.01)*randn(1);
    end    
end
x_test = n + b;

figure();
plot(time,x_gyro,'.');
hold on;
plot(time,x_test,'.r');
legend('Measured','Modeled');
xlabel('Time (sec)');
ylabel('Acceleration (m/s^2)');
title('X Axis Acceleration');
grid on;

break;

% Print out model

disp(['X acc: bias = ' num2str(x_acc_bias) ' sigma = ' num2str(std(x_acc))  ]);
disp(['Y acc: bias = ' num2str(y_acc_bias) ' sigma = ' num2str(std(y_acc))  ]);
disp(['Z acc: bias = ' num2str(z_acc_bias) ' sigma = ' num2str(std(z_acc))  ]);
disp(' ');
disp(['X gyro: bias = ' num2str(x_gyro_bias) ' sigma = ' num2str(std(x_gyro))  ]);
disp(['Y gyro: bias = ' num2str(y_gyro_bias) ' sigma = ' num2str(std(y_gyro))  ]);
disp(['Z gyro: bias = ' num2str(z_gyro_bias) ' sigma = ' num2str(std(z_gyro))  ]);
disp(' ');

%% Ploting

% Acceleromter Allan Deviation
figure();
loglog(tauxg,adxg,'.-b','LineWidth',2,'MarkerSize',24);
hold on;
loglog(tauyg,adyg,'.-r','LineWidth',2,'MarkerSize',24);
loglog(tauzg,adzg,'.-g','LineWidth',2,'MarkerSize',24);
legend('X acc','Y acc','Z acc');
grid on;
title(['Allan Deviation: Accelerometer '],'FontSize',FontSize+2,'FontName',FontName);
xlabel('\tau (sec)','FontSize',FontSize,'FontName',FontName);
ylabel('\sigma_y (m/s^2)','FontSize',FontSize,'FontName',FontName);

%  Gyroscope Allan Deviation
figure();
loglog(taux,adx,'.-b','LineWidth',2,'MarkerSize',24);
hold on;
loglog(tauy,ady,'.-r','LineWidth',2,'MarkerSize',24);
loglog(tauz,adz,'.-g','LineWidth',2,'MarkerSize',24);
legend('X gyro','Y gyro','Z gyro');
grid on;
title(['Allan Deviation: Gyroscope '],'FontSize',FontSize+2,'FontName',FontName);
xlabel('\tau (sec)','FontSize',FontSize,'FontName',FontName);
ylabel('\sigma_y (deg/s)','FontSize',FontSize,'FontName',FontName);

% Plot Accelermoter
figure();
subplot(3,1,1);
plot(time,x_acc,'.');
hold on;
plot(time,x_acc_mdl,'.r');
legend('Measured','Modeled');
xlabel('Time (sec)');
ylabel('Acceleration (m/s^2)');
title('X Axis Acceleration');
grid on;

subplot(3,1,2);
plot(time,y_acc,'.');
hold on;
plot(time,y_acc_mdl,'.r');
legend('Measured','Modeled');
xlabel('Time (sec)');
ylabel('Acceleration (m/s^2)');
title('Y Axis Acceleration');
grid on;

subplot(3,1,3);
plot(time,z_acc,'.');
hold on;
plot(time,z_acc_mdl,'.r');
legend('Measured','Modeled');
xlabel('Time (sec)');
ylabel('Acceleration (m/s^2)');
title('Z Axis Acceleration');
grid on;
suptitle('Acceleromter Modeling');

% Plot Gyroscope
figure();
subplot(3,1,1);
plot(time,x_gyro,'.');
hold on;
plot(time,x_gyro_mdl,'.r');
legend('Measured','Modeled');
xlabel('Time (sec)');
ylabel('Angular Velocity (rad/s)');
title('X Axis Angular Velocity');
grid on;

subplot(3,1,2);
plot(time,y_gyro,'.');
hold on;
plot(time,y_gyro_mdl,'.r');
legend('Measured','Modeled');
xlabel('Time (sec)');
ylabel('Angular Velocity (rad/s)');
title('Y Axis Angular Velocity');
grid on;

subplot(3,1,3);
plot(time,z_gyro,'.');
hold on;
plot(time,z_gyro_mdl,'.r');
legend('Measured','Modeled');
xlabel('Time (sec)');
ylabel('Angular Velocity (rad/s)');
title('Z Axis Angular Velocity');
grid on;

suptitle('Gyroscope Modeling');

% Plot Amplitude Spectrum
% [f_acc_x, amp_acc_x] = myftt(x_acc_nobias, IMUPer, 0);
% [f_acc_xm, amp_acc_xm] = myftt(x_acc_mdl-mean(x_acc_mdl), IMUPer, 0);
% 
% figure();
% title('Single-Sided Amplitude Spectrum');
% subplot(3,2,1);
% loglog(f_acc_x,amp_acc_x); 
% hold on;
% loglog(f_acc_xm,amp_acc_xm,'r'); 
% xlabel('Frequency (Hz)');
% ylabel('Acceleration (m/s^2)');
% grid on;








