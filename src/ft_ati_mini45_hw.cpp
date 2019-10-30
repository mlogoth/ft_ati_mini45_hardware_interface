#include <vector>
#include <string>
#include <iostream>

// ROS
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
// PCAN 
#include <libpcanfd.h>

using namespace hardware_interface;


class ATIMini45 : public RobotHW
{

public:

  ATIMini45()
  {

        /* Initialize Hardware Interface Class */

        // connect and register force/torque sensor interface
        ForceTorqueSensorHandle ft_handle("ft_sensor", _attached_link, frc, trq);
        ft_sensor_interface.registerHandle(ft_handle);
        registerInterface(&ft_sensor_interface);

        // ROS Node Handler
        n_ = ros::NodeHandle();


  }


  void initCommunication()
  {
    fd = pcanfd_open(_device, OFD_BITRATE, 500000);
  }

private:
  // Force Torque Interface
  ForceTorqueSensorInterface ft_sensor_interface;
  // Force and Torques
  double frc[3];
  double trq[3];
  // Attached Link
  std::string _attached_link = "world";
  // Node Handler
  ros::NodeHandle n_;
  // FD of Socket
  int fd;
  // Device CAN
  std::string _device = "/dev/pcan0";

};