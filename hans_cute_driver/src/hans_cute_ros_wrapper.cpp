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
  ROS_INFO("Robot driver started")
  follow_joint_as_.start();
  ROS_INFO("follow_joint_trajectory action server started");
}

void HansCuteRosWrapper::halt()
{
  start_ = false;
  driver_.halt();
  ROS_INFO("Robot driver halted")
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
  ROS_INFO_STREAM("Hans ROS Driver: goal received");
}

void HansCuteRosWrapper::followJointTrajCancelCb(
    const actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle &goal_handle)
{
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
    const trajectory_msgs::JointTrajectory &traj, const double &err) const
{
  std::unordered_map<std::string, double> joint_states;
  {
    std::unique_lock<std::mutex> lck(driver_mtx_);
    if (!driver_.getJointStates(joint_states))
    {
      return false;
    }
  }
  for (size_t i = 0; i < traj.points[0].positions.size(); ++i)
  {
    if (fabs(traj.points[0].positions[i] - q_act[i]) > err)
      return false;
  }
  return true;
}

bool HansCuteRosWrapper::

    int
    main(int argc, char **argv)
{
  ros::init(argc, argv, "hans_cute_ros_driver");
  ros::NodeHandle nh;
  HansCuteRosWrapper wrapper(nh);
  wrapper.init();
  wrapper.start();
  ros::spin();
  return 0;
}