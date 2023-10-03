// #include <iostream>

// #include "hans_cute_driver/hans_cute_driver.h"
// #include "hans_cute_driver/hans_cute_datatype.h"

// #include "serial_port/custom_serial_port.h"
// #include "serial_port/serial_port_interface.h"

// int main(int argc, char *argv[])
// {
//   std::shared_ptr<SerialPortInterface> serial_port = std::make_shared<SerialPort>("/dev/ttyUSB0", 250000, 50);
//   HansCuteRobot::ServoDriver hans_robot;
//   hans_robot.setSerialPort(serial_port);
//   hans_robot.open();

//   std::vector<unsigned int> servo_ids;
//   for (int i = 0; i <= 6; i++)
//   {
//     servo_ids.push_back(i);
//     hans_robot.setTorqueEnable(i, true);
//     hans_robot.setAcceleration(i, 20);
//     hans_robot.setSpeed(i,300);

//     unsigned int position = 0;
//     hans_robot.getPosition(i, position);
//     hans_robot.setPosition(i, position);
//     hans_robot.setPosition(i, 2048);
//   }
//   hans_robot.close();
//   return 0;
// }

// #include <stdio.h>
// #include <stdlib.h>
// #include <libudev.h>

// int main()
// {
//   struct udev *udev;
//   struct udev_device *device;
//   struct udev_enumerate *enumerate;
//   struct udev_list_entry *devices, *entry;

//   // Initialize udev context
//   udev = udev_new();
//   if (!udev)
//   {
//     fprintf(stderr, "Failed to initialize udev\n");
//     return 1;
//   }

//   // Create udev enumeration object
//   enumerate = udev_enumerate_new(udev);
//   if (!enumerate)
//   {
//     fprintf(stderr, "Failed to create udev enumeration\n");
//     udev_unref(udev);
//     return 1;
//   }

//   // Add a filter for USB devices with specific product and vendor IDs
//   udev_enumerate_add_match_subsystem(enumerate, "tty");
//   udev_enumerate_add_match_property(enumerate, "ID_VENDOR_ID", "1a86");
//   udev_enumerate_add_match_property(enumerate, "ID_MODEL_ID", "7523");

//   // Scan for devices
//   udev_enumerate_scan_devices(enumerate);

//   // Get a list of matching devices
//   devices = udev_enumerate_get_list_entry(enumerate);

//   // Iterate through the list of matching devices
//   udev_list_entry_foreach(entry, devices)
//   {
//     const char *syspath = udev_list_entry_get_name(entry);
//     device = udev_device_new_from_syspath(udev, syspath);

//     // Get the device node path (e.g., /dev/ttyUSB0)
//     const char *devnode = udev_device_get_devnode(device);
//     if (devnode)
//     {
//       printf("Device Node: %s\n", devnode);
//     }

//     udev_device_unref(device);
//   }

//   // Cleanup
//   udev_enumerate_unref(enumerate);
//   udev_unref(udev);

//   return 0;
// }

#include <iostream>
#include <libudev.h>
#include "hans_cute_driver/serial_port_manager.hpp"

int main()
{
  SerialPortManager manager;
  manager.startMonitoring();
  while(true)
  {
    std::cout << manager.serialPortAvailable("0403","6001") << std::endl;
  }
  return 0;
}
