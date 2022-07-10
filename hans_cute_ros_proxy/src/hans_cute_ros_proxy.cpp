#include "hans_cute_ros_proxy/hans_cute_ros_proxy.h"

HansCuteRosProxy::HansCuteRosProxy(const std::string& port_name, const std::string& port_namespace,
                                   const long& baud_rate, const unsigned int& min_motor_id,
                                   const unsigned int& max_motor_id)
  : port_name_(port_name)
  , port_namespace_(port_namespace)
  , baud_rate_(baud_rate)
  , min_motor_id_(min_motor_id)
  , max_motor_id_(max_motor_id)
  , running_(false)
{
}

HansCuteRosProxy::HansCuteRosProxy() : HansCuteRosProxy("/dev/ttyUSB", "ttyUSB0", 250000, 0, 7)
{
}

HansCuteRosProxy::~HansCuteRosProxy()
{
}

void HansCuteRosProxy::setServoDriver(std::shared_ptr<HansCuteRobot::ServoDriver> servo_driver_ptr)
{
  servo_driver_ptr_ = servo_driver_ptr;
}

bool HansCuteRosProxy::connect()
{
  // Find motors

  // Start 2 update threads
}

bool HansCuteRosProxy::disconnect()
{
  findMotors();

  if (running_)
  {
  }
}

bool HansCuteRosProxy::findMotors()
{
  std::vector<unsigned int> motor_ids_list;
  unsigned int num_retries = 5;
  for (int servo_id = min_motor_id_; servo_id <= max_motor_id_; servo_id++)
  {
    for (int ping_try = 1; ping_try <= num_retries; ping_try++)
    {
      if (ping_try == num_retries)
      {
        std::cout << "Failed to ping motor: " << servo_id << std::endl;
        break;
      }

      // We should wrap this ping function in driver maybe
      std::vector<uint8_t> response;
      servo_driver_ptr_->ping((uint8_t)servo_id, response);
      if (response.size() == 0)
      {
        continue;
      }

      // IF we can ping this servo then, we should be able to retrieve the servo params
      bool servo_params_filled = false;
      for (int query_param_try = 1; query_param_try <= num_retries; query_param_try++)
      {
        if (query_param_try == num_retries)
        {
          std::cout << "Unable to retrieve servo params for servo: " << servo_id << std::endl;
          break;
        }

        unsigned int model_number = 0;
        if (servo_driver_ptr_->getModelNumber(servo_id, model_number))
        {
          continue;
        }
        fillMotorParameters(servo_id, model_number);
        servo_params_filled = true;
        break;
      }

      // Only add to id list if we can retrieve params of this servo
      if (servo_params_filled)
      {
        motor_ids_list.push_back(servo_id);
      }

      break;
    }
  }

  if (motor_ids_list.size() == 0)
  {
    std::cout << "Unable to find any servo from the given ids" << std::endl;
    return false;
  }

  return true;
}

bool HansCuteRosProxy::fillMotorParameters(const unsigned int& servo_id, const unsigned int& model_number)
{
}