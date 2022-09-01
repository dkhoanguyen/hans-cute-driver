# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased-0.0.2] - 2022-09-01
### Added
- Add hans_cute_driver responsibile for sending and interpreting raw serial packet from the robot servo motors [dkhoanguyen]
- Add hans_cute_bringup for launching the controller node and loading configurations to parameter server [dkhoanguyen]
- Add a sample configuration for the driver [dkhoanguyen]
- Separate `serial_command` from `hans_cute_driver` and make it a standalone package [dkhoanguyen]
- Add a simple `JointPositionController` in `hans_cute_controller` to enable joint position controller using ROS Topic [dkhoanguyen]
- Add some MATLAB example scripts outlining how to control the robot through ROS and MATLAB
- Add README [dkhoanguyen]
- Add docker support for the package [dkhoanguyen]

### Removed
- Remove `hans_cute_ros_proxy(or hans_cute_status_manager)` and move the responsibility of finding and configuring the servos to a new class in `hans_cute_driver` called `HansCuteRobot` [dkhoanguyen]

### Changed
- Update TODO [dkhoanguyen]
- Update CHANGELOG [dkhoanguyen]

## [Unreleased-0.0.1] - 2022-07-31
### Added
- Add hans_cute_driver responsibile for sending and interpreting raw serial packet from the robot servo motors [dkhoanguyen]
- Add hans_cute_ros_proxy responsible for querying, monitoring and publishing robot status to ros [dkhoanguyen]
- Add CHANGELOG for keeping track of changes to this project [dkhoanguyen]
- Add TODO for keeping track of the important features and changes needed for this project [dkhoanguyen]