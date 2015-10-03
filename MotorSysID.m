% Wil Selby
% Washington, DC
% Aug 31, 2015

% This script processes .mat files comprised of ArduPilot Mavlink messages
% converted from .tlog files to .mat files in Mission Planner. This script
% will model the accelermoter, gyroscope, and GPS sensors

%% Initialization

clear all;
close all;
clc;

% Add Paths
addpath logs

%% Load log
load MotorRPM;

%% Extract data

t_start = 300;
t_end = 780;

time = MotorRPM(t_start:t_end,1)/1e3;
PWM = MotorRPM(t_start:t_end,3);
RPM = MotorRPM(t_start:t_end,5)/2; % 2 blades/propellor

% Remove RPM < 2500 (sensor errors)
RPMinds = RPM >= 2500;

time = time(RPMinds);
PWM = PWM(RPMinds);
RPM = RPM(RPMinds);

%% Simulate System

tau = 0.15;
DC = 6.5;

s = tf('s');
G = DC/(tau*s+1);

dt = 0.01;
t = time(1):dt:time(end);

for(i = 1:size(t,2))
    if (t(i) > 4.5 && t(i) <= 6.5)
        PWM_sim(i) = 1000;
    end
    if (t(i) > 6.5 && t(i) <= 8.5)
        PWM_sim(i) = 1425;
    end
    if (t(i) > 8.5 && t(i) <= 10.5)
        PWM_sim(i) = 1475;
    end
    if (t(i) > 10.5 && t(i) <= 12.5)
        PWM_sim(i) = 1450;
    end
    if (t(i) > 12.5 && t(i) <= 14.5)
        PWM_sim(i) = 1390;
    end
    if (t(i) > 14.5 && t(i) <= 16.5)
        PWM_sim(i) = 1425;
    end
    if (t(i) > 16.5 && t(i) <= 18.5)
        PWM_sim(i) = 1480;
    end
    if (t(i) > 18.5 && t(i) <= 20.5)
        PWM_sim(i) = 1425;
    end
    if (t(i) > 20.5 &&t(i) <= 22.5)
        PWM_sim(i) = 1375;
    end
    if (t(i) > 22.5)
        PWM_sim(i) = 1000;
    end
end

RPM_sim = lsim(G,(PWM_sim),t);

%% Discretized model

% Using simulated PWM commands
sysD = c2d(G,dt,'zoh');

for(k = 1:size(t,2))
    if k == 1
        RPM_est(k,1) = 0;
    end
    if k > 1
        RPM_est(k,1) = .9355*RPM_est(k-1,1) + .4192*PWM_sim(k-1);
    end
end

% Using recorded PWM commands
RCPer = median(diff(time));
sysD = c2d(G,RCPer,'zoh');

for(k = 1:size(time,1))
    if k == 1
        RPM_est2(k,1) = 0;
    end
    if k > 1
        RPM_est2(k,1) = .8133*RPM_est2(k-1,1) + 1.214*PWM(k-1);
    end
end

RPM_est2 = RPM_est2-5525;
%% Plot

% figure();
% plot(time,PWM);
% hold on;
% plot(t,PWM_sim,'r');

figure();
plot(time,RPM,'r');
hold on;
% plot(t,RPM_sim-5525,'b');
% plot(t,RPM_est-5525,'g');
plot(time,RPM_est2,'k');
axis([8.5,22,3300,4200])
str = sprintf('Simulated vs Actual Motor Dynamics (DC = %.2f Tc = %.2f)',DC,tau);
title(str);
xlabel('Time (s)');
ylabel('RPM');
legend('Actual','Simulated');




