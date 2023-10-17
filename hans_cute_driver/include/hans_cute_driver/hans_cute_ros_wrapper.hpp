#ifndef HANS_CUTE_DRIVER__HANS_CUTE_ROS_WRAPPER_HPP_
#define HANS_CUTE_DRIVER__HANS_CUTE_ROS_WRAPPER_HPP_

#include <thread>
#include <atomic>
#include <algorithm>
#include <unordered_map>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommand.h>
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
  ros::Publisher gripper_state_pub_;
  ros::ServiceServer home_ss_;
  ros::ServiceServer teach_ss_;

  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_as_;
  actionlib::ActionServer<control_msgs::GripperCommandAction> gripper_command_as_;
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle follow_traj_goal_handle_;
  actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle gripper_goal_handle_;


  // FollowJointTrajectory
  void followJointTrajGoalCb(
      const actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle &goal_handle);
  void followJointTrajCancelCb(
      const actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle &goal_handle);

  // GripperCommand
  void gripperCommandCb(
      const actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle &goal_handle);
  void gripperCommandCancelCb(
      const actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle &goal_handle);

  bool homingCb(std_srvs::TriggerRequest &req,
                std_srvs::TriggerResponse &res);

  void stateThread(const ros::TimerEvent &event);
  bool hasPoints(const trajectory_msgs::JointTrajectory &traj);
  bool isStartPositionsMatch(
      const trajectory_msgs::JointTrajectory &traj, const double &err);
  void goalTrajControlThread(const trajectory_msgs::JointTrajectory &traj);
  void gripperCommandThread(const control_msgs::GripperCommand &command);

protected:
  std::atomic<bool> start_;
  std::atomic<bool> has_goal_;
  std::atomic<bool> has_gripper_goal_;
  std::atomic<bool> pause_follow_joint_traj_as_;

  std::mutex driver_mtx_;
  HansCuteRobot::HansCuteDriver driver_;

  bool cancelCurrentGoal();
  bool constructNameJointMapping(const std::vector<std::string> &names,
                                 const std::vector<double> &pos,
                                 std::unordered_map<std::string, double> &output);
  bool isAtGoal(const std::unordered_map<std::string, double> &current_pos,
                const std::unordered_map<std::string, double> &goal_pos,
                const double &err);

  std::vector<double> computeError(const std::unordered_map<std::string, double> &current_pos,
                                   const std::unordered_map<std::string, double> &goal_pos);
  bool jointNamesValid(const std::vector<std::string> &joint_names);
};

#endif