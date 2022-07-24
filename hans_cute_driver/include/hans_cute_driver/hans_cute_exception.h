#ifndef _HANS_CUTE_EXCEPTIONS_H_
#define _HANS_CUTE_EXCEPTIONS_H_

#include <stdexcept>

namespace HansCuteRobot
{
class SerialOpenError : public std::runtime_error
{
public:
  SerialOpenError(const std::string& port, const long& baudrate, const std::string& message)
    : std::runtime_error(message)
  {
    _msg = "Cannot open port " + port + "at" + std::to_string(baudrate) + "with error: " + message;
  }
  ~SerialOpenError() throw()
  {
  }
  const char* what() const throw() override
  {
    return _msg.c_str();
  }

private:
  std::string _msg;
};

class ChecksumError : public std::runtime_error
{
public:
  ChecksumError(const uint8_t& id, const uint8_t response_checksum, const uint8_t& checksum, const std::string& message)
    : std::runtime_error(message)
  {
    _msg = "Mismatch checksum (" + std::to_string(response_checksum) + "!=" + std::to_string(checksum) + ") from motor " + std::to_string(id);
  }
  ~ChecksumError() throw()
  {
  }
  const char* what() const throw() override
  {
    return _msg.c_str();
  }

private:
  std::string _msg;
};
};  // namespace HansCuteRobot
#endif