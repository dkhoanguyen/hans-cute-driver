#include "controller_manager.h"

HansCuteControllerManager::HansCuteControllerManager(ros::NodeHandle &nh, const std::string &port) : nh_(nh), rate_(10)
{
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

  status_manager_ptr_->updateJointParams(joint_params);

  // Controller
  joint_controller_ptr_ = std::make_shared<HansCuteController::JointPositionController>(servo_driver_ptr_,
                                                                                        node_namespace_,
                                                                                        port_namespace);

  joint_controller_ptr_->setJointNames(joint_names);

  // Joint IDS
  std::vector<unsigned int> joint_ids;
  status_manager_ptr_->getJointIds(joint_ids);
  joint_controller_ptr_->setJointIds(joint_ids);

  // Joint Params
  std::vector<ServoParams> joint_param;
  status_manager_ptr_->getJointParameters(joint_param);
  joint_controller_ptr_->setServoParams(joint_param);

  // Initialise
  joint_controller_ptr_->initialise();

  // Start
  joint_controller_ptr_->start();
}