clear all;
clc;
close all;
rosshutdown;
%% Start Hans Cute Node
rosinit;

%% Start Dobot ROS
hans = HansCute();

%% Test Motion
%% Publish custom joint target
jointTargets = [0,0,0,0,0,0,0]; % 7 joint positions
hans.PublishTargetJoint(jointTargets);

%% Publish gripper state
gripperState = 500;
hans.PublishGripperState(gripperState);