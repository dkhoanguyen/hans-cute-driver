#include "controller_manager.h"

HansCuteControllerManager::HansCuteControllerManager(ros::NodeHandle &nh) : nh_(nh), rate_(10)
{
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  joint_target_sub_ = nh_.subscribe("target_joint_states", 10, &HansCuteControllerManager::jointTargetCallback, this);
}

HansCuteControllerManager::~HansCuteControllerManager()
{
}

void HansCuteControllerManager::initialise()
{
  // This function should be separated into smaller sections
  // Load params from ros
  std::string port = "/dev/ttyUSB0"; // Default port
  int baud_rate = 250000;            // Default baud rate

  // Get Namespace and node name first
  node_namespace_ = ros::this_node::getNamespace();
  node_name_ = ros::this_node::getName().substr(node_namespace_.size());

  // Load port and baud rate
  if (!(nh_.getParam(node_name_.substr(1) + "/port", port)))
  {
    ROS_ERROR("No port specified.");
  }
  if (!(nh_.getParam(node_name_.substr(1) + "/baud_rate", baud_rate)))
  {
    ROS_ERROR("No baudrate specified.");
  }

  std::string port_namespace = port.substr(5); // Remove /dev/ from port name
  // Shared pointer for hardware driver
  servo_driver_ptr_ = std::make_shared<HansCuteRobot::ServoDriver>(port, baud_rate);
  servo_driver_ptr_->open();

  status_manager_ptr_ = std::make_shared<HansCuteStatusManager>();
  status_manager_ptr_->setServoDriver(servo_driver_ptr_);
  status_manager_ptr_->initialise();

  // Joint Params
  std::vector<std::string> joint_names;
  std::vector<ServoParams> joint_params;
  for (unsigned int id = 0; id < 6; id++)
  {
    // Params
    ServoParams joint_param;

    // Get Joint Name first
    std::string joint_name = "joint_" + std::to_string(id);
    if (!(nh_.getParam(node_name_.substr(1) + "/robot_hardware/joint_params/" + joint_name + "/name", joint_name)))
    {
      ROS_ERROR("Unable to retrieve name from parameter server");
    }
    joint_names.push_back(joint_name);
    joint_param.joint_name = joint_name;

    // Get other joint params
    int raw_origin = 2048;
    int raw_min = 853;
    int raw_max = 3243;

    // Joint origin and min max positions
    if (!(nh_.getParam(node_name_.substr(1) + "/robot_hardware/joint_params/" + joint_name + "/origin", raw_origin)))
    {
      ROS_ERROR("Unable to retrieve raw origin from parameter server");
    }
    joint_param.raw_origin = raw_origin;

    if (!(nh_.getParam(node_name_.substr(1) + "/robot_hardware/joint_params/" + joint_name + "/min", raw_min)))
    {
      ROS_ERROR("Unable to retrieve raw min angle from parameter server");
    }
    joint_param.raw_min = raw_min;

    if (!(nh_.getParam(node_name_.substr(1) + "/robot_hardware/joint_params/" + joint_name + "/max", raw_max)))
    {
      ROS_ERROR("Unable to retrieve raw max angle from parameter server");
    }
    joint_param.raw_max = raw_max;

    // Joint speed and accelaration
    int speed = 1023;
    int acceleration = 1023;

    if (!(nh_.getParam(node_name_.substr(1) + "/robot_hardware/joint_params/" + joint_name + "/speed", speed)))
    {
      ROS_ERROR("Unable to retrieve speed from parameter server");
    }
    joint_param.raw_origin = raw_origin;

    if (!(nh_.getParam(node_name_.substr(1) + "/robot_hardware/joint_params/" + joint_name + "/acceleration", acceleration)))
    {
      ROS_ERROR("Unable to retrieve acceleration from parameter server");
    }
    joint_param.raw_origin = raw_origin;
    joint_params.push_back(joint_param);
  }

  // Update user defined parameters with robot hardware
  status_manager_ptr_->updateJointParams(joint_params);

  // Controller
  controller_ptr_ = std::make_shared<HansCuteController::JointPositionController>(servo_driver_ptr_,
                                                                                  node_namespace_,
                                                                                  port_namespace);

  controller_ptr_->setJointNames(joint_names);

  // Joint IDS
  std::vector<unsigned int> joint_ids;
  status_manager_ptr_->getJointIds(joint_ids);
  controller_ptr_->setJointIds(joint_ids);

  // Joint Params
  std::vector<ServoParams> joint_param;
  status_manager_ptr_->getJointParameters(joint_param);
  controller_ptr_->setServoParams(joint_param);

  // Initialise
  status_manager_ptr_->start();
  controller_ptr_->start();
}

void HansCuteControllerManager::start()
{
  control_thread_ = std::make_unique<std::thread>(std::thread(&HansCuteControllerManager::controlThread, this));
}

void HansCuteControllerManager::stop()
{
}

void HansCuteControllerManager::jointTargetCallback(const trajectory_msgs::JointTrajectoryConstPtr &joint_traj_msg)
{
  std::unique_lock<std::mutex> lck(target_joint_buff_.mtx);
  target_joint_buff_.data_deq.push_back(*joint_traj_msg);
  if (target_joint_buff_.data_deq.size() > 10)
  {
    target_joint_buff_.data_deq.pop_front();
  }
  target_joint_buff_.received = true;
}

void HansCuteControllerManager::controlThread()
{
  ROS_INFO("HansCuteControllerManager: Control thread started.");
  while (ros::ok())
  {
    if (target_joint_buff_.received)
    {
      HansCuteController::Data joint_pos_data;
      std::unique_lock<std::mutex> lck(target_joint_buff_.mtx);
      joint_pos_data.set(target_joint_buff_.data_deq.back().points.at(0).positions);
      controller_ptr_->processCommand(joint_pos_data);
      target_joint_buff_.received = false;
    }
  }
  rate_.sleep();
}