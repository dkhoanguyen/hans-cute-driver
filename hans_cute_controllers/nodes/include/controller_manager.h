#ifndef _CONTROLLER_MANAGER_H_
#define _CONTROLLER_MANAGER_H_

#include <cmath>
#include <deque>
#include <thread>

#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "hans_cute_controllers/controller.h"
#include "hans_cute_status_manager/hans_cute_status_manager.h"
#include "hans_cute_controllers/joint_position_controller.h"
#include "hans_cute_driver/hans_cute_driver.h"

struct JointDataBuffer
{
  std::deque<sensor_msgs::JointState> joint_deq;
  std::mutex mtx;
  std::atomic<bool> received;
};

class HansCuteControllerManager
{
public:
  HansCuteControllerManager(ros::NodeHandle &nh, const std::string &port);
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
  std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr_;
  std::shared_ptr<HansCuteController::Controller> joint_controller_ptr_;
  std::shared_ptr<HansCuteStatusManager> status_manager_ptr_;

  JointDataBuffer joint_state_buff_;
  JointDataBuffer target_joint_buff_;

  std::string node_name_;
  std::string node_namespace_;
};

#endif