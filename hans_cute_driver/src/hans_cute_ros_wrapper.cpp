#include "hans_cute_driver/hans_cute_ros_wrapper.hpp"

HansCuteRosWrapper::HansCuteRosWrapper(ros::NodeHandle &nh)
    : nh_(nh), start_(false), has_goal_(false),
      follow_joint_as_(nh, "/follow_joint_trajectory",
                       boost::bind(&HansCuteRosWrapper::followJointTrajGoalCb, this, _1),
                       boost::bind(&HansCuteRosWrapper::followJointTrajCancelCb, this, _1),
                       false)
{
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  state_thread_ = nh_.createTimer(ros::Duration(0.05), &HansCuteRosWrapper::stateThread, this);
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
    ;
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
  ROS_INFO("Init done");
  manager.stopMonitoring();
}

void HansCuteRosWrapper::start()
{
  start_ = true;
  driver_.start();
  ROS_INFO("Robot driver started");
  follow_joint_as_.start();
  ROS_INFO("follow_joint_trajectory action server started");
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
  ROS_INFO_STREAM("Hans_ROS_Driver: goal received");
  auto goal = *(goal_handle.getGoal());
  goal_handle_ = goal_handle;
  if (has_goal_)
  {
    ROS_WARN_NAMED("Hans_ROS_Driver", "Received new goal while still executing previous trajectory. Canceling previous trajectory");
    result_.error_code = -100;
    result_.error_string = "Hans_ROS_Driver: Received another trajectory";
    ROS_ERROR_STREAM(result_.error_string);
    goal_handle_.setAborted(result_, result_.error_string);
    // Handle control goal cancellation here
    cancelCurrentGoal();
    return;
  }

  if (!hasPoints(goal.trajectory))
  {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string = "Hans_ROS_Driver: Received trajectory has no point. Rejecting...";
    ROS_ERROR_STREAM(result_.error_string);
    goal_handle_.setRejected(result_, result_.error_string);
    return;
  }

  // Accept goal
  goal_handle_.setAccepted();
  has_goal_ = true;
  boost::thread(boost::bind(&HansCuteRosWrapper::goalTrajControlThread, this, goal.trajectory)).detach();
}

void HansCuteRosWrapper::followJointTrajCancelCb(
    const actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle &goal_handle)
{
}

void HansCuteRosWrapper::goalTrajControlThread(const trajectory_msgs::JointTrajectory &traj)
{
  // We can safely assume that the first point of the traj is the current pos of the arm
  // Therefore, trajectory starts from the second one
  int current_indx = 1;
  // Get all joint names
  std::vector<std::string> joint_names = traj.joint_names;
  // We add roughly 1 seconds offset
  double total_duration = traj.points.end()->time_from_start.toSec() + 1;
  ros::Time start_time = ros::Time::now();
  while (ros::ok() && current_indx < traj.points.size())
  {
    // Calculate the elapsed time
    ros::Time current_time = ros::Time::now();
    double elapsed_time = (current_time - start_time).toSec();

    // Check if the elapsed time exceeds the desired duration
    if (elapsed_time >= total_duration)
    {
      ROS_WARN_NAMED("Hans ROS Driver", "Execution time exceeded the desired duration.");
      break;
    }

    trajectory_msgs::JointTrajectoryPoint point = traj.points.at(current_indx);
    double exec_time = point.time_from_start.toSec();

    // Obtain current position first
    std::unordered_map<std::string, double> current_joint_states;
    {
      std::unique_lock<std::mutex> lck(driver_mtx_);
      if (!driver_.getJointStates(current_joint_states))
      {
        ROS_ERROR("Unable to query joint states. Reusing the previously available one");
      }
    }
    // Constructing a map name -> joint
    std::unordered_map<std::string, double> target_joint_goals;
    std::unordered_map<std::string, double> target_joint_vels;
    constructNameJointMapping(joint_names, point.positions, target_joint_goals);
    // Calculate the velocity required for each joint
    for (std::string joint_name : joint_names)
    {
      double vel = std::abs(target_joint_goals.at(joint_name) - current_joint_states.at(joint_name)) / exec_time;
      target_joint_vels[joint_name] = vel;
    }

    // Send command
    {
      std::unique_lock<std::mutex> lck(driver_mtx_);
      driver_.setJointPVT(target_joint_goals, target_joint_vels);
    }

    // Continously check for the current joint and compare it with the goal
    do
    {
      std::unordered_map<std::string, double> current_joint_states;
      {
        std::unique_lock<std::mutex> lck(driver_mtx_);
        if (!driver_.getJointStates(current_joint_states))
        {
          ROS_ERROR("Unable to query joint states. Reusing the previously available one");
        }
      }
    } while (!isAtGoal(current_joint_states, target_joint_goals, 0.001));
    // Robot at target, move to the next point
    current_indx++;
  }

  // Execution done
  if (has_goal_)
  {
    result_.error_code = result_.SUCCESSFUL;
    goal_handle_.setSucceeded(result_);
    has_goal_ = false;
  }
}

// Utils
bool HansCuteRosWrapper::hasPoints(
    const trajectory_msgs::JointTrajectory &traj)
{
  if (traj.points.size() == 0)
    return false;
  for (auto &point : traj.points)
  {
    if (point.positions.size() != traj.joint_names.size() ||
        point.velocities.size() != traj.joint_names.size())
      return false;
  }
  return true;
}

bool HansCuteRosWrapper::isStartPositionsMatch(
    const trajectory_msgs::JointTrajectory &traj, const double &err)
{
  return true;
}
bool HansCuteRosWrapper::cancelCurrentGoal()
{
  std::unordered_map<std::string, double> joint_states;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    if (!driver_.getJointStates(joint_states))
    {
      ROS_ERROR_NAMED("Hans ROS Driver", "Unable to query joint states to cancel current goal");
      return false;
    }
    else
    {
      // Only cancel goal if we can query joint state
      if (!driver_.setJointPTP(joint_states, 0.2, 0.2))
      {
        ROS_ERROR_NAMED("Hans ROS Driver", "Unable to cancel current goal");
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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hans_cute_ros_driver");
  ros::NodeHandle nh;
  HansCuteRosWrapper wrapper(nh);
  wrapper.init();
  wrapper.start();
  ros::spin();
  return 0;
}