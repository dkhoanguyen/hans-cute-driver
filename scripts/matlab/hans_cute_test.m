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
 joint_target = [0,0,0,0,0,0,0];
hans.PublishTargetJoint(joint_target);

%% Publish gripper state
gripper_state = 500;
hans.PublishGripperState(gripper_state);