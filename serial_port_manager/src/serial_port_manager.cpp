#include "serial_port_manager/serial_port_manager.hpp"

SerialPortManager::SerialPortManager(ros::NodeHandle nh)
    : nh_(nh), start_monitoring_(false)
{
  udev_ = udev_new();
  if (!udev_)
  {
    std::cerr << "Failed to initialize udev" << std::endl;
    return;
  }

  monitor_ = udev_monitor_new_from_netlink(udev_, "udev");
  udev_monitor_filter_add_match_subsystem_devtype(monitor_, "tty", NULL);
  udev_monitor_enable_receiving(monitor_);

  // ROS stuff
  monitor_thread_ = nh_.createTimer(ros::Duration(0.1), &SerialPortManager::monitorThread, this);
}
SerialPortManager::~SerialPortManager()
{
}

bool SerialPortManager::serialPortAvailable(const std::string &vendor_id, const std::string &product_id, std::string &path)
{
  std::vector<std::string> output;
  for (const auto &pair : device_map_)
  {
    if (pair.second == vendor_id + ":" + product_id)
    {
      output.push_back(pair.first);
    }
  }
  if (output.size() == 1)
  {
    path = output.at(0);
    return true;
  }
  else if (output.size() == 0)
  {
    return false;
  }

  // Check all ports to see if any of them are occupied and return the first available one
  for (std::string port : output)
  {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd != -1)
    {
      close(fd);
      path = port;
      return true;
    }
    close(fd);
  }
  return false;
}

void SerialPortManager::startMonitoring()
{
  // Scan for all existing devices before monitoring
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices, *entry;
  ;

  // Create an enumerate object to list devices in the "tty" subsystem
  // In the future we should also monitor for other types of devices as well
  enumerate = udev_enumerate_new(udev_);
  udev_enumerate_add_match_subsystem(enumerate, "tty");
  udev_enumerate_scan_devices(enumerate);

  // Get a list of devices
  devices = udev_enumerate_get_list_entry(enumerate);

  // Iterate through the list and print device paths, vendor ID, and product ID
  udev_list_entry_foreach(entry, devices)
  {
    const char *device_path = udev_list_entry_get_name(entry);
    struct udev_device *device = udev_device_new_from_syspath(udev_, device_path);
    const char *devnode = udev_device_get_devnode(device);

    // Get the vendor ID and product ID from device properties
    const char *vendor_id = udev_device_get_property_value(device, "ID_VENDOR_ID");
    const char *product_id = udev_device_get_property_value(device, "ID_MODEL_ID");
    const char *device_typ = udev_device_get_property_value(device, "ID_TYPE");

    if (product_id && vendor_id)
    {
      device_map_[devnode] = std::string(vendor_id) + ":" + std::string(product_id);
    }
    udev_device_unref(device);
  }

  // Cleanup
  udev_enumerate_unref(enumerate);

  // When done start monitoring threads
  start_monitoring_ = true;
}

void SerialPortManager::stopMonitoring()
{
  start_monitoring_ = false;
}

void SerialPortManager::monitorThread(const ros::TimerEvent &event)
{
  if (!start_monitoring_)
  {
    return;
  }

  struct udev_device *device = udev_monitor_receive_device(monitor_);
  if (device)
  {
    const char *action = udev_device_get_action(device);
    const char *devnode = udev_device_get_devnode(device);

    if (action && devnode)
    {
      if (std::string(action) == "add")
      {
        auto it = device_map_.find(std::string(devnode));
        if (it == device_map_.end())
        {
          // Extract vendor and product IDs from udev properties
          const char *vendor_id = udev_device_get_property_value(device, "ID_VENDOR_ID");
          const char *product_id = udev_device_get_property_value(device, "ID_MODEL_ID");

          if (vendor_id && product_id)
          {
            // Use the device node as the key and store vendor/product IDs as a pair in the unordered_map
            device_map_[devnode] = std::string(vendor_id) + ":" + std::string(product_id);
          }
        }
      }
      else if (std::string(action) == "remove")
      {
        // Device removed, remove it from the list
        auto it = device_map_.find(std::string(devnode));
        if (it != device_map_.end())
        {
          device_map_.erase(devnode);
          // std::cout << "Removed Device: " << devnode << std::endl;
        }
      }
      udev_device_unref(device);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_port_manager");
  return 0;
}