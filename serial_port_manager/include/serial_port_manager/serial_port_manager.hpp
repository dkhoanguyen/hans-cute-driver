#ifndef SERIAL_PORT_MANAGER__SERIAL_PORT_MANAGER_HPP_
#define SERIAL_PORT_MANAGER__SERIAL_PORT_MANAGER_HPP_

#include <iostream>
#include <unordered_map>
#include <vector>
#include <thread>
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <libudev.h>

#include <ros/ros.h>

class SerialPortManager
{
public:
  SerialPortManager(ros::NodeHandle nh);
  ~SerialPortManager();

  bool serialPortAvailable(const std::string &vendor_id, const std::string &product_id, std::string &path);
  void startMonitoring();
  void stopMonitoring();

protected:
  ros::NodeHandle nh_;
  ros::Timer monitor_thread_;

  void monitorThread(const ros::TimerEvent &event);

protected:
  std::atomic<bool> start_monitoring_;
  struct udev *udev_;
  struct udev_monitor *monitor_;

  std::vector<std::string> device_list_;
  std::unordered_map<std::string, std::string> device_map_;
  
};

#endif