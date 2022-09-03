# Hans Cute Robot ROS Driver

## Introduction
The hans-cute-driver provides a bare minimum interface to communicate with the Hans Cute (Cyton) Robot. This driver also leverages ROS as well as other ROS native tools to create an end-to-end system where a user can control and query data from the robot in a relatively straightforward manner. However, it does not mean that ROS is required to use this driver, as it is developed with the goal of being a standalone library for any c++ project.

## Currently supported features

- Joint State query and Joint Position Control using ROS Topics
- Gripper State query and Gripper Control using ROS Topics

## Contents

This repository contains the following packages:
- `serial_command_robot`: A bare minimum interface for designing robot manipulators that use serial bus as their primary communicating interface
- `hans_cute_driver`: A driver designed specifically for the hans cute robot. It communicates with the XQtor Servo on each joint of the robot using a modified version of the Dynamixel Protocol 1.0.
- `hans_cute_controller`: A controller package designed specifically for controlling the hans cute robot, using custom controllers and also "ros control" controllers
- `hans_cute_bringup`: A package contains the startup script and configuration files for the hans cute robot
- `docker`: This folder contains some dockerfile to support for development, testing, and in the future, software packaging and deployment.
- `scripts`: This folder contains some simple scripts written in MATLAB and Python showing how to control the robot through ROS using ROS-supported languages.

## Installation
### For end user

### For developers (and Robotics students)

Simply clone the repository in the `src` directory of your catkin_workspace
```
cd ~/catkin_ws/src
git clone https://github.com/dkhoanguyen/hans-cute-driver
```
Then catkin_make normally
```
cd ..
catkin_make
```
Remember to source your workspace after `catkin_make`
```
source devel/setup.bash
```
If you are running into problems with your USB connection, either because of system permission, or SerialPort unable open the usb connection, you may want to try this command WITH YOUR USB CABLE CONNECTED:
```
sudo chown $LOGNAME /dev/ttyUSB0
```

## Quick Start
### Preparing the robot
When receiving the robot, please make sure that you check for any loose screw or misplaced joints, using the provided documents and images. Then put the robot down in a flat and stable surface. If possible, it is recommened to use tape or any other means to stablise the robot base onto the surface, as the force generated during its operation may cause the base to displace unintentionally.

### Preparing the ROS computer

Install ROS then install the package using the above instruction.

Run the following command to start the package
```
roslaunch hans_cute_bringup hans_cute_bringup.launch
```

### Sending control commands using MATLAB
#### Joint Position
Since the hans has 7 joints, please ensure that when sending control commands there must be exactly 7 joint values, similar to the MATLAB example script. Joint position value range depends on the min and max values listed in the configuration file and the unit is radian.
```
%% Start Dobot ROS
hans = HansCute();

%% Test Motion
%% Publish custom joint target
jointTargets = [0,0,0,0,0,0,0]; % 7 joint positions
hans.PublishTargetJoint(jointTargets);
```
#### Gripper
You can control the width of the gripper claw by sending a raw position value, ranges from 200 to 500 (according to the default values in the configuration files).
```
%% Start Dobot ROS
hans = HansCute();

%% Publish gripper state
gripperState = 500;
hans.PublishGripperState(gripperState);
```

## How to configure the driver
In the `hans_cute_bringup` package there is a `config` folder that stores all yaml files for configuring the driver with user-defined parameters. Users can also add their own custom config yaml files to this folder if they wish, but please make sure to adjust the launch file to point to the correct config file.

Right now, the default config file is `hans_cute_joint_pos_controller.yml`, which has the configuration for the Joint Position Controller. Users can modify the following parameters to meet the requirements of their applications:
- `origin`: This is the raw origin position for each joint of the robot. You can increase or decrease this value to modify the starting position of the robot, especially in a screnario where the robot does not appear to be standing up straight when all joints are set to 0. However, the origin must not exceed the `min` `max` joint limits or `0` and `4095` if the limits were not specified.
  
- `speed`: This is the raw speed for each joint of the robot. You can increase of decrease this value to make each joint rotates slower or faster. The value ranges from `0` to `1023`. Please note that it is recommened to keep this value within the range of 300 to 500 max as it is not safe to use the robot when it is moving with joint speeds greater than 500.

- `acceleration`: This is the raw acceleration for each joint of the robot. You can increase or decrease, range from `0` to `1023` to adjust the jerkiness of each joint when the robot moves. Please also keep this value from within 30 to 50 max.

NOTE: Please do not modify other parameters that are not listed in the above lists, as it may affect some basic functionalities of the driver.

## Troubleshooting
TBA