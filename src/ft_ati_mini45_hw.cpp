#include <vector>
#include <string>
#include <iostream>
#include <malloc.h>
// ROS
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/console.h>
// PCAN 
#include <libpcanfd.h>

using namespace hardware_interface;


#define RATE 50.0

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

  
  void setCANMsg(struct pcanfd_msg *msg, std::string _id, unsigned short _data_len, std::string string ) {
    

    if(_data_len>PCANFD_MAXDATALEN || string.size()>PCANFD_MAXDATALEN){
      ROS_ERROR("Wrong Msg Set");
      exit(-1);
    }

    msg->id = std::stoul(_id, nullptr,16);
    msg->data_len = _data_len;
    msg->flags = PCANFD_MSG_RTR;
    msg->type = PCANFD_TYPE_CAN20_MSG;

    //fill data of msg
    for (unsigned int  i=0;i<string.size();i++) {
            msg->data[i]=string[i];
    }
    for (unsigned int i=string.size(); i<PCANFD_MAXDATALEN; i++) {
        msg->data[i]='0';
    }
    std::cout<< "ID: "<< msg->id <<std::endl;
    std::cout<< "Data Length: "<< msg->data_len <<std::endl;
    std::cout<< "Data: " << msg->data <<std::endl;
  };

  void initCommunication()
  {
    // Opens a CAN 2.0 channel for reading and writing @1Mb bitrate,
    // accepting extended format messages too
    fd = pcanfd_open(_device, OFD_BITRATE, 1000000);
    fd = 1 ;
    if (fd < 0) 
    {
      ROS_ERROR("Could Not Open CAN Device: %s",_device);
      exit(-1);
    }

    else
    {
      ROS_INFO("Succesfully Open CAN Device: %s",_device);
      /* 
      * Initialize CAN Device
      */

      // Create a pointer of an array of msgs
      struct pcanfd_msgs *pml;
      // Allocate Enough Room to store 6 CAN Messages
      pml = (struct pcanfd_msgs*) malloc(sizeof(*pml)+6*sizeof(struct pcanfd_msg));
      
      if (!pml){
        ROS_ERROR("Memory Allocation Failed");
        exit(-1);
      }

      pml->count = 6; // we will send 6 msgs

      /*
      *  | 20Fh |  6 | 00 00 00 00 00 01  | Wait  |
      *  | 20Fh |  6 | 2D 00 00 00 00 01  | Wait  |
      *  | 20Fh |  6 | 0D 11 0D 40 0D 41  | Wait  |
      *  | 20Fh |  6 | 0D 12 00 00 00 01  | Wait  |
      *  | 20Fh |  6 | 2D 11 0D 40 0D 41  | Wait  |
      *  | 20Fh |  6 | 2D 12 00 00 00 01  | Wait  |
      */
      std::vector<std::string> InitData = {"000000000001","2D0000000001","0D110D400D41","0D120D400D41","2D110D400D41","2D1200000001"};
     
      // Fill msgs
      for (pml->count=0; pml->count<6; pml->count++) {
        ATIMini45::setCANMsg(pml->list + pml->count,"0x20f",6, InitData[pml->count]);
      }
      
      

      
      exit(-1);
      // for (pml->count=0; pml->count<6;pml->count++) {
      //   pml->list->
      // }


    }


  }

  void read(){ 
    ROS_INFO("Read Function");
  }

  void write(){
    ROS_INFO("Write Function");
  }

  ros::Time get_time() { return ros::Time::now(); } ;
  ros::Duration get_period() { ros::Duration(1.0/RATE);};

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
  const char* _device = "/dev/pcan0";

};





int main(int argc, char **argv)
{
  ros::init(argc, argv, "FTATIMini45_hw_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(RATE);

  ATIMini45 robot;

  //initialize control manager
  controller_manager::ControllerManager cm(&robot,nh);

  //Initialize Variables
  robot.initCommunication();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration dur = robot.get_period();

  while (ros::ok())
  {
      ros::spinOnce();


      //time loop starts
      ros::Time start =ros::Time::now();

      std::cout << "+++++++++++++++++++++++++++++++++++++++++"<<std::endl;
      // read robot data
      robot.read();

      // update data
      cm.update(start, dur);

      //send data to the robot
      robot.write();

      
      //loop_rate.sleep();

      loop_rate.sleep();
      // time loop ends
      dur = robot.get_time()-start;

      //print dt
      printf("Loop dt:%lf\n", dur.toSec());
      std::cout << "+++++++++++++++++++++++++++++++++++++++++"<<std::endl;

  }

 return 0;
}
