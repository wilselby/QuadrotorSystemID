%% Initialization
clc;
addpath matlab_rosbag
clear rosbag_wrapper;
clear ros.Bag;
clear all
close all


% dir_str = '~/.ros/';
% d = dir(sprintf('%s*.bag',dir_str));
% [dx,dy] = sort([d.datenum]);
% newest = d(dy(end)).name;
% oldest = d(dy(1)).name;
%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
% load_path = sprintf('%s%s',dir_str,newest);
% bag = ros.Bag.load('~/matlab_ws/multi_agent/bag_files/MAR20/crazy_data__2015-03-20-08-39-26.bag');

% load_path = sprintf('~/catkin_ws/bag_files/%s',newest);
load_path = 'uav_id_data__2015-07-02-15-54-45.bag';
bag = ros.Bag.load(load_path);

%bag = ros.Bag.load('/home/wseuser/matlab_rosbag/example/example.bag');

bag.info()
%% Read all messages on a few topics
uav_name = 'Quadrotor';
pose_topic = sprintf('/vicon/%s/Body',uav_name);
rcin_topic = '/mavros/rc/in';
rcout_topic = '/mavros/rc/out';
imu_topic = '/mavros/imu/data';

%% ArduPilot Logs

% RC Inputs
[msgs, meta] = bag.readAll(rcin_topic); % Messages are structs
accessor = @(x) x.channels;
RC_in = ros.msgs2mat(msgs,accessor); % Convert struct to 3-by-N matrix of linear velcoity
t_RC_in = cellfun(@(x) x.time.time, meta); % Get timestamps

% PWM Outputs
[msgs, meta] = bag.readAll(rcout_topic); % Messages are structs
accessor = @(x) x.channels;
pwm_out = ros.msgs2mat(msgs,accessor); % Convert struct to 3-by-N matrix of linear velcoity
t_pwm_out = cellfun(@(x) x.time.time, meta); % Get timestamps

% IMU mesaurements
[msgs, meta] = bag.readAll(imu_topic); % Messages are structs
accessor = @(x) x.orientation;
quat = ros.msgs2mat(msgs,accessor); % Convert struct to 3-by-N matrix of linear velcoity
[yaw, pitch, roll] = quat2angle(quat);
accessor = @(x) x.angular_velocity;
gyro = ros.msgs2mat(msgs,accessor); % Convert struct to 3-by-N matrix of linear velcoity
accessor = @(x) x.linear_acceleration;
accel = ros.msgs2mat(msgs,accessor); % Convert struct to 3-by-N matrix of linear velcoity
t_imu = cellfun(@(x) x.time.time, meta); % Get timestamps

%% Vicon Information

% Translation
[msgs, meta] = bag.readAll(pose_topic); % Messages are structs
accessor = @(x) x.transform.translation;
pos = ros.msgs2mat(msgs,accessor); % Convert struct to 3-by-N matrix of linear velcoity
t_pos = cellfun(@(x) x.time.time, meta); % Get timestamps

%Rotation
accessor = @(x) x.transform.rotation;
att = ros.msgs2mat(msgs,accessor); % Convert struct to 3-by-N matrix of linear velcoity
t_att = cellfun(@(x) x.time.time, meta); % Get timestamps

%% Plot
figure
plot(pos(1,:),pos(2,:))








