#ifndef HANS_CUTE_DRIVER__HANS_CUTE_ROS_WRAPPER_HPP_
#define HANS_CUTE_DRIVER__HANS_CUTE_ROS_WRAPPER_HPP_

#include <thread>
#include <atomic>
#include <unordered_map>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#include "hans_cute_driver/serial_port_manager.hpp"
#include "hans_cute_driver.hpp"

class HansCuteRosWrapper
{
public:
  HansCuteRosWrapper(ros::NodeHandle &nh);
  ~HansCuteRosWrapper();

  void init();
  void start();
  void halt();

protected:
  ros::NodeHandle nh_;
  ros::Timer state_thread_;
  ros::Publisher joint_state_pub_;

protected:
  std::atomic<bool> start_;
  std::mutex driver_mtx_;
  HansCuteRobot::HansCuteDriver driver_;
  void stateThread(const ros::TimerEvent &event);
};

#endif