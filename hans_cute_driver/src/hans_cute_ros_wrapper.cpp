#include "hans_cute_driver/hans_cute_ros_wrapper.hpp"

HansCuteRosWrapper::HansCuteRosWrapper(ros::NodeHandle &nh)
    : nh_(nh), start_(false), has_goal_(false),
      has_gripper_goal_(false), pause_follow_joint_traj_as_(false),
      follow_joint_as_(nh, "/follow_joint_trajectory",
                       boost::bind(&HansCuteRosWrapper::followJointTrajGoalCb, this, _1),
                       boost::bind(&HansCuteRosWrapper::followJointTrajCancelCb, this, _1),
                       false),
      gripper_command_as_(nh, "/gripper_command",
                          boost::bind(&HansCuteRosWrapper::gripperCommandCb, this, _1),
                          boost::bind(&HansCuteRosWrapper::gripperCommandCancelCb, this, _1),
                          false)
{
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  state_thread_ = nh_.createTimer(ros::Duration(0.05), &HansCuteRosWrapper::stateThread, this);
  home_ss_ = nh_.advertiseService("/home", &HansCuteRosWrapper::homingCb, this);
}

HansCuteRosWrapper::~HansCuteRosWrapper()
{
}

void HansCuteRosWrapper::init()
{
  // Init robot
  // SerialPortManager should be a rosnode
  SerialPortManager manager;
  manager.startMonitoring();
  while (manager.serialPortAvailable("0403", "6001").empty())
  {
  }
  std::string port = manager.serialPortAvailable("0403", "6001");
  driver_.init("/dev/ttyUSB0");

  //  Get Namespace and node name first
  std::string node_namespace = ros::this_node::getNamespace();
  std::string node_name = ros::this_node::getName();
  std::string usb_device = "1a86:7523";
  int baudrate = 250000;
  // Load port and baud rate
  if (!(nh_.getParam(node_name + "/config/comms/usb_device", usb_device)))
  {
    ROS_ERROR("No device specified.");
  }
  if (!(nh_.getParam(node_name + "/config/comms/baudrate", baudrate)))
  {
    ROS_ERROR("No baudrate specified.");
  }

  // Robot joints
  for (unsigned int id = 0; id <= 6; id++)
  {
    // Get Joint Name first
    std::string joint_name = "joint_" + std::to_string(id);
    if (!(nh_.getParam(node_name + "/config/servo_params/" + "joint_" + std::to_string(id) + "/name", joint_name)))
    {
      ROS_ERROR("Unable to retrieve name from parameter server");
    }

    // Get other joint params
    int raw_origin = 2048;
    int raw_min = 853;
    int raw_max = 3243;

    // Joint origin and min max positions
    if (!(nh_.getParam(node_name + "/config/servo_params/" + "joint_" + std::to_string(id) + "/origin", raw_origin)))
    {
      ROS_ERROR("Unable to retrieve raw origin from parameter server");
    }

    if (!(nh_.getParam(node_name + "/config/servo_params/" + "joint_" + std::to_string(id) + "/min", raw_min)))
    {
      ROS_ERROR("Unable to retrieve raw min angle from parameter server");
    }

    if (!(nh_.getParam(node_name + "/config/servo_params/" + "joint_" + std::to_string(id) + "/max", raw_max)))
    {
      ROS_ERROR("Unable to retrieve raw max angle from parameter server");
    }

    // Joint speed and accelaration
    int speed = 300;
    int acceleration = 20;

    if (!(nh_.getParam(node_name + "/config/servo_params/" + "joint_" + std::to_string(id) + "/speed", speed)))
    {
      ROS_ERROR("Unable to retrieve speed from parameter server");
    }

    if (!(nh_.getParam(node_name + "/config/servo_params/" + "joint_" + std::to_string(id) + "/acceleration", acceleration)))
    {
      ROS_ERROR("Unable to retrieve acceleration from parameter server");
    }

    driver_.setJointLimits(
        "joint_" + std::to_string(id),
        joint_name, raw_min, raw_max,
        raw_origin, speed, acceleration);
  }

  // Robot gripper
  // Get Joint Name first
  std::string joint_name = "gripper";
  if (!(nh_.getParam(node_name + "/config/servo_params/gripper/name", joint_name)))
  {
    ROS_ERROR("Unable to retrieve name from parameter server");
  }

  // Get other joint params
  int raw_origin = 300;
  int raw_min = 100;
  int raw_max = 500;
  int speed = 300;
  int acceleration = 20;

  if (!(nh_.getParam(node_name + "/config/servo_params/gripper/min", raw_min)))
  {
    ROS_ERROR("Unable to retrieve raw min angle from parameter server");
  }

  if (!(nh_.getParam(node_name + "/config/servo_params/gripper/max", raw_max)))
  {
    ROS_ERROR("Unable to retrieve raw max angle from parameter server");
  }
  // driver_.setJointLimits("gripper", joint_name, raw_min, raw_max,
  //                        raw_origin, speed, acceleration);
  ROS_INFO("Init done");
  manager.stopMonitoring();
}

void HansCuteRosWrapper::start()
{
  start_ = true;
  driver_.start();
  ROS_INFO("Robot driver started");
  follow_joint_as_.start();
  ROS_INFO("FollowJointTrajectory action server started");
  gripper_command_as_.start();
  ROS_INFO("GripperCommand action server started");
}

void HansCuteRosWrapper::halt()
{
  start_ = false;
  driver_.halt();
  ROS_INFO("Robot driver halted");
}

void HansCuteRosWrapper::stateThread(const ros::TimerEvent &event)
{
  sensor_msgs::JointState joint_state_msg;
  std::unordered_map<std::string, double> joint_states;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    if (!driver_.getJointStates(joint_states))
    {
      ROS_ERROR("Unable to query joint states");
      return;
    }
  }
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  for (const auto &joint_state : joint_states)
  {
    joint_names.push_back(joint_state.first);
    joint_positions.push_back(joint_state.second);
    joint_velocities.push_back(0.0);
  }

  joint_state_msg.name = joint_names;
  joint_state_msg.position = joint_positions;
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_pub_.publish(joint_state_msg);
}

void HansCuteRosWrapper::followJointTrajGoalCb(
    const actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle &goal_handle)
{
  if (pause_follow_joint_traj_as_)
  {
    ROS_ERROR("Hans_ROS_Driver: FollowJointTrajectory is paused atm. Rejecting incoming goal...");
    return;
  }

  control_msgs::FollowJointTrajectoryResult result;
  ROS_INFO_STREAM("Hans_ROS_Driver: goal received");
  auto goal = *(goal_handle.getGoal());
  follow_traj_goal_handle_ = goal_handle;

  if (has_goal_)
  {
    ROS_WARN("Hans_ROS_Driver: Received new goal while still executing previous trajectory. Canceling previous trajectory");
    result.error_code = -100;
    result.error_string = "Hans_ROS_Driver: Received another trajectory";
    follow_traj_goal_handle_.setRejected(result, result.error_string);
    cancelCurrentGoal();
    has_goal_ = false;
    return;
  }

  if (!hasPoints(goal.trajectory))
  {
    result.error_code = result.INVALID_GOAL;
    result.error_string = "Hans_ROS_Driver: Received trajectory has no point. Rejecting...";
    ROS_ERROR_STREAM(result.error_string);
    follow_traj_goal_handle_.setRejected(result, result.error_string);
    return;
  }

  if (!jointNamesValid(goal.trajectory.joint_names))
  {
    result.error_code = result.INVALID_JOINTS;
    result.error_string = "Hans_ROS_Driver: Received trajectory has invalid or unknown joint names. Rejecting...";
    ROS_ERROR_STREAM(result.error_string);
    follow_traj_goal_handle_.setRejected(result, result.error_string);
    return;
  }

  // Check for joint limits bound

  // Accept goal
  follow_traj_goal_handle_.setAccepted();
  has_goal_ = true;
  boost::thread(boost::bind(&HansCuteRosWrapper::goalTrajControlThread, this, goal.trajectory)).detach();
}

void HansCuteRosWrapper::followJointTrajCancelCb(
    const actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle &goal_handle)
{
  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = -200;
  result.error_string = "Hans ROS Driver: Goal Cancellation received. Cancelling current goal now...";
  follow_traj_goal_handle_.setCanceled(result, result.error_string);
  cancelCurrentGoal();
}

void HansCuteRosWrapper::goalTrajControlThread(const trajectory_msgs::JointTrajectory &traj)
{
  // We can safely assume that the first point of the traj is the current pos of the arm
  // Therefore, trajectory starts from the second one
  int current_indx = 1;
  // Get all joint names
  std::vector<std::string> joint_names = traj.joint_names;
  double total_duration = traj.points.at(traj.points.size() - 1).time_from_start.toSec();
  ros::Time start_time = ros::Time::now();

  // Constructing a map name -> joint
  std::unordered_map<std::string, double> target_joint_goals;
  std::unordered_map<std::string, double> target_joint_vels;

  control_msgs::FollowJointTrajectoryResult result;

  while (ros::ok() && current_indx < traj.points.size())
  {
    // Check if goal is still valid
    if (follow_traj_goal_handle_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
    {
      ROS_INFO("Hans ROS Driver: Current goal handle is not active. Stop execution");
      return;
    }

    trajectory_msgs::JointTrajectoryPoint point = traj.points.at(current_indx);
    double exec_time = point.time_from_start.toSec() + 0.5;
    // Obtain current position first
    std::unordered_map<std::string, double> current_joint_states;
    {
      std::unique_lock<std::mutex> lck(driver_mtx_);
      if (!driver_.getJointStates(current_joint_states))
      {
        ROS_ERROR("Unable to query joint states. Reusing the previously available one");
      }
    }

    // Obtain current goal point
    constructNameJointMapping(joint_names, traj.points.at(current_indx).positions, target_joint_goals);
    // Calculate the velocity required for each joint
    for (std::string joint_name : joint_names)
    {
      double vel = std::abs(target_joint_goals[joint_name] - current_joint_states[joint_name]) / exec_time;
      target_joint_vels[joint_name] = vel;
    }

    // Send command
    {
      ROS_INFO("Hans ROS Driver: Sending trajectory");
      std::unique_lock<std::mutex> lck(driver_mtx_);
      if (!driver_.setJointPVT(target_joint_goals, target_joint_vels))
      {
        ROS_ERROR_NAMED("Hans ROS Driver", "Unable to set control command to the robot");
      }
    }

    while (ros::ok())
    {
      // Check if goal is still valid
      if (follow_traj_goal_handle_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      {
        ROS_INFO("Hans ROS Driver: Current goal handle is not active. Stop execution");
        return;
      }

      // Calculate the elapsed time
      ros::Time current_time = ros::Time::now();
      double elapsed_time = (current_time - start_time).toSec();

      // Check if the elapsed time exceeds the desired duration
      if (elapsed_time >= total_duration)
      {
        ROS_WARN_NAMED("Hans ROS Driver", "Execution time exceeded the desired duration. Aborting current goal");
        cancelCurrentGoal();
        result.error_code = result.GOAL_TOLERANCE_VIOLATED;
        result.error_string = "";
        follow_traj_goal_handle_.setAborted(result);
        return;
      }

      // Continously check for the current joint and compare it with the goal
      std::unordered_map<std::string, double> current_joint_states;
      {
        std::unique_lock<std::mutex> lck(driver_mtx_);
        if (!driver_.getJointStates(current_joint_states))
        {
          ROS_ERROR("Hans ROS Driver: Unable to query joint states");
        }
      }

      // Publishing feedback
      control_msgs::FollowJointTrajectoryFeedback feedback;
      feedback.header.stamp = ros::Time::now();
      feedback.joint_names = traj.joint_names;
      feedback.desired = traj.points.at(current_indx);
      for (auto joint_state : current_joint_states)
      {
        feedback.actual.positions.push_back(joint_state.second);
      }
      follow_traj_goal_handle_.publishFeedback(feedback);

      // Check if at target
      bool at_target = isAtGoal(current_joint_states, target_joint_goals, 0.05);
      if (at_target)
      {
        // Robot at target, move to the next point
        ROS_INFO("Hans ROS Driver: Robot at target, move to the next point");
        current_indx++;
        break;
      }
    }
  }
  // Execution done
  if (has_goal_)
  {
    ROS_INFO("Hans ROS Driver: Trajectory execution completed");
    result.error_code = result.SUCCESSFUL;
    result.error_string = "";
    follow_traj_goal_handle_.setSucceeded(result);
    has_goal_ = false;
  }
}

void HansCuteRosWrapper::gripperCommandCb(
    const actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle &goal_handle)
{
  control_msgs::GripperCommandResult result;
  gripper_goal_handle_ = goal_handle;
  if (has_gripper_goal_)
  {
    ROS_WARN("Hans_ROS_Driver: Received new command while still executing previous one. Canceling previous command");
    gripper_goal_handle_.setRejected();
    has_gripper_goal_ = false;
    return;
  }
  gripper_goal_handle_.setAccepted();
  has_gripper_goal_ = true;
  auto goal = *gripper_goal_handle_.getGoal();
  boost::thread(boost::bind(&HansCuteRosWrapper::gripperCommandThread, this, goal.command)).detach();
}
void HansCuteRosWrapper::gripperCommandCancelCb(
    const actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle &goal_handle)
{
}

bool HansCuteRosWrapper::homingCb(std_srvs::TriggerRequest &req,
                                  std_srvs::TriggerResponse &res)
{
  // Check if action server is executing any goal
  if (has_goal_)
  {
    ROS_ERROR("Han ROS Driver: FollowJointTrajectory Action Server is executing a goal. Cancelling this request");
    res.success = false;
    res.message = "Han ROS Driver: FollowJointTrajectory Action Server is executing a goal. Cancelling this request";
    return false;
  }

  ROS_INFO("Hans ROS Driver: Homing service called. Executing...");
  // Prevent action server from accepting goals
  pause_follow_joint_traj_as_ = true;
  std::unordered_map<std::string, double> homing_goal;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    if (!driver_.getJointStates(homing_goal))
    {
      ROS_ERROR("Hans ROS Driver: Unable to query joint states to cancel current goal");
      return false;
    }

    for (auto goal : homing_goal)
    {
      homing_goal[goal.first] = 0.0;
    }

    // Only cancel goal if we can query joint state
    if (!driver_.setJointPTP(homing_goal, 0.1, 1.0))
    {
      ROS_ERROR("Hans ROS Driver: Unable to set homing command");
      res.success = false;
      res.message = "Hans ROS Driver: Unable to set homing command";
      pause_follow_joint_traj_as_ = false;
      return false;
    }
  }

  // Wait until it is homed
  while (ros::ok())
  {
    // Continously check for the current joint and compare it with the goal
    std::unordered_map<std::string, double> current_joint_states;
    {
      std::unique_lock<std::mutex> lck(driver_mtx_);
      if (!driver_.getJointStates(current_joint_states))
      {
        ROS_ERROR("Hans ROS Driver: Unable to query joint states");
      }
    }

    // Check if at target
    bool at_target = isAtGoal(current_joint_states, homing_goal, 0.05);
    if (at_target)
    {
      break;
    }
  }

  // Process the request and set the response fields
  res.success = true;
  res.message = "Homing executed successfully!";
  pause_follow_joint_traj_as_ = false;
  return true;
}

void HansCuteRosWrapper::gripperCommandThread(const control_msgs::GripperCommand &command)
{
  control_msgs::GripperCommandResult result;
  double pos = command.position;
  // Send command
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    driver_.setGripperCommand(pos);
  }
  // Ensure that the gripper is at location
  ros::Time start_time = ros::Time::now();
  double total_duration = 5;
  while (ros::ok())
  {
    // Get current gripper position
    double current_pos = 0.0;
    {
      std::unique_lock<std::mutex> lck(driver_mtx_);
      driver_.getGripperPos(current_pos);
    }

    double elapsed_time = (ros::Time::now() - start_time).toSec();
    // Check if the elapsed time exceeds the desired duration
    if (elapsed_time >= 5)
    {
      result.effort = 0;
      result.position = current_pos;
      result.reached_goal = false;
      result.stalled = true;

      gripper_goal_handle_.setAborted(result);
      has_gripper_goal_ = false;
      return;
    }

    // Publish feedback
    control_msgs::GripperCommandFeedback feedback;
    feedback.stalled = false;
    feedback.position = current_pos;
    feedback.effort = 1.5;
    feedback.reached_goal = false;
    gripper_goal_handle_.publishFeedback(feedback);

    if (current_pos - pos <= 0.001)
    {
      break;
    }
  }
  // Gripper reaches position
  double current_pos = 0.0;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    driver_.getGripperPos(current_pos);
  }
  result.effort = 1.5;
  result.position = current_pos;
  result.reached_goal = true;
  result.stalled = false;

  gripper_goal_handle_.setSucceeded(result);
  has_gripper_goal_ = false;
}

// Utils
bool HansCuteRosWrapper::hasPoints(
    const trajectory_msgs::JointTrajectory &traj)
{
  if (traj.points.size() == 0)
    return false;
  for (auto &point : traj.points)
  {
    if (point.positions.size() != traj.joint_names.size())
      return false;
  }
  return true;
}

bool HansCuteRosWrapper::isStartPositionsMatch(
    const trajectory_msgs::JointTrajectory &traj, const double &err)
{
  std::unordered_map<std::string, double> current_joint_states;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    if (!driver_.getJointStates(current_joint_states))
    {
      ROS_ERROR("Unable to query joint states. Reusing the previously available one");
    }
  }
  std::unordered_map<std::string, double> target_joint_goals;
  for (auto joint_pos : current_joint_states)
  {
    // std::string joint_name = joint_pos.first;
    // if (std::abs(joint_pos.second - goal_pos.at(joint_name)) > err)
    // {
    //   return false;
    // }
  }
  return true;
}
bool HansCuteRosWrapper::cancelCurrentGoal()
{
  std::unordered_map<std::string, double> joint_states;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    if (!driver_.getJointStates(joint_states))
    {
      ROS_ERROR("Hans ROS Driver: Unable to query joint states to cancel current goal");
      return false;
    }
    else
    {
      // Only cancel goal if we can query joint state
      if (!driver_.setJointPTP(joint_states, 0.2, 0.2))
      {
        ROS_ERROR("Hans ROS Driver: Unable to cancel current goal");
        return false;
      }
    }
  }
  // Temporary sleep, should check for current pos
  boost::this_thread::sleep_for(boost::chrono::milliseconds(250));
  return true;
}

bool HansCuteRosWrapper::isAtGoal(const std::unordered_map<std::string, double> &current_pos,
                                  const std::unordered_map<std::string, double> &goal_pos,
                                  const double &err)
{
  for (auto joint_pos : current_pos)
  {
    std::string joint_name = joint_pos.first;
    if (std::abs(joint_pos.second - goal_pos.at(joint_name)) > err)
    {
      return false;
    }
  }
  return true;
}

bool HansCuteRosWrapper::constructNameJointMapping(const std::vector<std::string> &names,
                                                   const std::vector<double> &pos,
                                                   std::unordered_map<std::string, double> &output)
{
  if (names.size() != pos.size())
  {
    return false;
  }
  for (auto idx = 0; idx < names.size(); idx++)
  {
    output[names.at(idx)] = pos.at(idx);
  }
  return true;
}

std::vector<double> HansCuteRosWrapper::computeError(const std::unordered_map<std::string, double> &current_pos,
                                                     const std::unordered_map<std::string, double> &goal_pos)
{
}

bool HansCuteRosWrapper::jointNamesValid(const std::vector<std::string> &joint_names)
{
  std::vector<std::string> joint_names_from_driver;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    driver_.getJointNames(joint_names_from_driver);
  }

  if (joint_names.size() != joint_names_from_driver.size())
  {
    return false;
  }
  // Sort
  std::vector<std::string> sorted_input_names = joint_names;
  std::vector<std::string> sorted_names_from_driver = joint_names_from_driver;
  std::sort(sorted_input_names.begin(), sorted_input_names.end());
  std::sort(sorted_names_from_driver.begin(), sorted_names_from_driver.end());

  // Compare the sorted vectors element by element
  for (int i = 0; i < sorted_input_names.size(); i++)
  {
    if (sorted_input_names.at(i) != sorted_names_from_driver.at(i))
    {
      return false; // Found a mismatch.
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hans_cute_ros_driver");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  HansCuteRosWrapper wrapper(nh);
  wrapper.init();
  wrapper.start();
  spinner.start();
  ros::waitForShutdown();
  return 0;
}