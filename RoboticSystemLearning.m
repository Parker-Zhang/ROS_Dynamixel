%% clear workspace
clear 
close all
clc

%% Initialize ROS
%if rosmaster run at ip '192.168.1.123',command below create a node
%communite with ros master
% rosinit('192.168.1.123') rosmaster run at ip '192.168.1.123'
% if no arguments are provided to rosinit ,Matlab will create both the ros
% master and a global node.
rosinit

%% Create the RobotSimulator
% this should open a window showing a robot within a map
simu = RobotSimulator

%% rosnode
rosnode list

%% rostopic
rostopic list 

%% get information about /turtle1/pose
rostopic info /turtle1/pose 

%% see what data is published on the topic
vel = rostopic('echo','/turtle1/pose')
showdetails(vel)
%% rosmsg show
rosmsg show turtlesim/Pose

%% create a subscriber for /turtle1/pose topic
scanner = rossubscriber('/turtle1/pose')

%% Three Way of accessing data
%% 1. Get the next message that arrives
data = receive(scanner,5)
%% 2. Get the latest data that received
data = scanner.LatestMessage
%% set an asynchronous callback for new message

%% Delete callback
scanner.NewMessageFcn = [] ;

%% visualize data
figure;
plot(data.X)
%% create a publisher to control the robot
turtleVelCmd = rospublisher('/turtle1/cmd_vel')

%% declare msg type
vel = rosmessage(turtleVelCmd)

%% define msg value & send it ???turtle not move??

send(turtleVelCmd,vel)

%% Get all available services 
rosservice list

%% shutdown ros
rosshutdown

