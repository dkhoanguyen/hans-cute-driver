#ifndef _CONTROLLER_MANAGER_H_
#define _CONTROLLER_MANAGER_H_

#include <cmath>
#include <deque>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "custom_serial_port/custom_serial_port.h"
#include "custom_serial_port/serial_port_interface.h"

#include "hans_cute_controllers/controller.h"
#include "hans_cute_status_manager/hans_cute_status_manager.h"
#include "hans_cute_controllers/joint_position_controller.h"
#include "hans_cute_driver/hans_cute_driver.h"

struct JointStateDataBuffer
{
  std::deque<sensor_msgs::JointState> data_deq;
  std::mutex mtx;
  std::atomic<bool> received;
};

struct JointTrajectoryDataBuffer
{
  std::deque<trajectory_msgs::JointTrajectory> data_deq;
  std::mutex mtx;
  std::atomic<bool> received;
};

class HansCuteControllerManager
{
public:
  HansCuteControllerManager(ros::NodeHandle &nh);
  ~HansCuteControllerManager();

  void initialise();
  void start();
  void stop();

private:
  void jointTargetCallback(const trajectory_msgs::JointTrajectoryConstPtr &joint_traj_msg);

  // Ros
  ros::NodeHandle nh_;
  ros::Rate rate_;

  // Publisher
  ros::Publisher joint_state_pub_;

  // Subscriber
  ros::Subscriber joint_target_sub_;

private:
  void controlThread();

  std::unique_ptr<std::thread> control_thread_;

  std::shared_ptr<SerialPortInterface> serial_port_ptr_;
  std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr_;
  std::shared_ptr<HansCuteController::Controller> controller_ptr_;
  std::shared_ptr<HansCuteStatusManager> status_manager_ptr_;

  JointStateDataBuffer joint_state_buff_;
  JointTrajectoryDataBuffer target_joint_buff_;

  std::string node_name_;
  std::string node_namespace_;

  std::atomic<bool> running_;
};

#endif