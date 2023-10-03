#ifndef HANS_CUTE_DRIVER__SERIAL_PORT_MANAGER_HPP_
#define HANS_CUTE_DRIVER__SERIAL_PORT_MANAGER_HPP_

#include <iostream>
#include <unordered_map>
#include <vector>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <libudev.h>

class SerialPortManager
{
public:
  SerialPortManager();
  ~SerialPortManager();

  std::string serialPortAvailable(const std::string &vendor_id, const std::string &product_id);
  void startMonitoring();

protected:
  struct udev *udev_;
  struct udev_monitor *monitor_;
  std::thread monitor_thread_;

  std::vector<std::string> device_list_;
  std::unordered_map<std::string, std::string> device_map_;
  void monitorFunc();
};

#endif