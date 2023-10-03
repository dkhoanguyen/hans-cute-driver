#ifndef HANS_CUTE_DRIVER__HANS_STATUS_HPP_
#define HANS_CUTE_DRIVER__HANS_STATUS_HPP_

namespace HansCuteRobot
{
  class HansStatus
  {
  public:
    HansStatus();
    ~HansStatus();

    //===============================//
    // Servo status access functions //
    //===============================//
    bool getModelNumber(const int &servo_id, unsigned int &model_number);
    bool getFirmwareVersion();
    bool getReturnDelayTime();

    bool getMaxTorque(const int &servo_id, unsigned int &torque);
    bool getTorqueEnabled(const int &servo_id, bool &enabled);
    // bool getAngleLimits(const int &servo_id, HansCuteRobot::AngleLimits &angle_limit);
    bool getVoltageLimits();

    bool getPosition(const int &servo_id, unsigned int &position);
    bool getSpeed(const int &servo_id, unsigned int &speed);
    bool getAcceleration(const int &servo_id, unsigned int &acceleration);

    bool getVoltage();
    bool getCurrent();
    bool getLock(const int &servo_id, bool &lock);

    // bool getFeedback(const int &servo_id, ServoFeedback &feedback);
  };
};

#endif