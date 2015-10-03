% Wil Selby
% Washington, DC
% September 08, 2015

% This script reads in data from the serial port. This is used to read data
% from the ArduPilot board into data structures for MATLAB post processing.

%% Initialization

clear all;
close all;
clc;

% Set up serial port
s = serial('COM4', 'BaudRate', 115200);

% Open serial port
fopen(s);

% 100Hz for 3 minutes ~ 18000 samples
for i=1:18000
    
    % ("Time: %f \t Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f \t
    % Gyro X:%4.2f \t Y:%4.2f \t Z:%4.2f\n")    
    % IMUdata(i) = fscanf(s,'%d,%d,%d,%d,%d,%d,%d,%d');
    str = fgetl(s);
    num = textscan(str, '%f', 'Delimiter',',');
end

%
fclose(s);
    
 