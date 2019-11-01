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
#include <libpcan.h>

using namespace hardware_interface;

bool DEBUG = true;

template <typename T>
std::string int_to_hex(T i)
{
  std::stringstream stream;
  stream << "0x"
         << std::setfill('0') << std::setw(sizeof(T))
         << std::hex << i;
  return stream.str();
}

#define RATE 1.0

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

  void printCANMSG(TPCANMsg *_msg){
    std::cout << " | Data: \n | ";
    for (int elt = 0; elt < 8; elt++) {
        std::cout << int_to_hex((int)_msg->DATA[elt]) << " ";
      }
    std::cout << "\n | LENGTH: " << int_to_hex((int)_msg->LEN) << " - " << (int)_msg->LEN << std::endl;
    std::cout << " | ID: " << int_to_hex((int)_msg->ID) << " - " << (int)_msg->ID << std::endl;
    std::cout << " | TYPE: " << (int)_msg->MSGTYPE << std::endl;

  }

  void setCANMsg(TPCANMsg *_msg, int _id, int _data_len, std::string _data)
  {
    if (_data_len > 8 || _data.size()/2 != _data_len)
    {
      ROS_ERROR("Wrong Msg Definition");
      exit(-1);
    }
    // Fill MSG ID
    _msg->ID = _id; //std::stoul(_id, nullptr,16);//0x020f;//std::stoul(_id, nullptr,16);//
    // Fill MSG Length
    _msg->LEN = (__u8)_data_len;
    _msg->MSGTYPE = MSGTYPE_STANDARD;

    //fill data of msg
    unsigned int cnt = 0;
    for (unsigned int i=0;i<_data.size();i+=2) {
      _msg->DATA[cnt] = (__u8)std::stoul(_data.substr(i,2), nullptr,16);
      cnt++;
    }
    // fill with zero values the rest of DATA
    while(cnt<8)
    {
      _msg->DATA[cnt] = (__u8)0x00;
      cnt++;
    }
    
    if (DEBUG) ATIMini45::printCANMSG(_msg); 
  }

  /*
  * INITIALIZATION FUNCTION
  */
  void initCommunication()
  {
    ROS_INFO("INITIALIZATION");
    // Opens a CAN 2.0 channel for reading and writing @1Mb bitrate,
    // accepting extended format messages too

    h = LINUX_CAN_Open(_device, 02);
    // Initialize the CAN Hardware with 1MBit/sec
    DWORD status = CAN_Init(h, CAN_BAUD_1M, CAN_INIT_TYPE_EX);

    // Send The Proper Messages for Initialization
    TPCANMsg msg, msg_r;
    DWORD status_write;


    std::cout << " ---- 1st INIT MSG ----" << std::endl;
    ATIMini45::setCANMsg(&msg,0x20f,6,"000000000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;


    std::cout << " ---- 2nd INIT MSG  ----" << std::endl;
    ATIMini45::setCANMsg(&msg,0x20f,6,"200000000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;


    std::cout << " ---- 3rd INIT MSG  ----" << std::endl;
    ATIMini45::setCANMsg(&msg,0x20f,6,"0D110D400D41");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;


    std::cout << " ---- 4th INIT MSG  ---- " << std::endl;
    ATIMini45::setCANMsg(&msg,0x20f,6,"0D1200000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;


    std::cout << " ---- 5th INIT MSG  ---- " << std::endl;
    ATIMini45::setCANMsg(&msg,0x20f,6,"2D110D400D41");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;


    std::cout << " ---- 6th INIT MSG  ---- " << std::endl;
    ATIMini45::setCANMsg(&msg,0x20f,6,"2D1200000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;

    sleep(1);
    
    std::cout<<"....";
    ATIMini45::setCANMsg(&msg,0x080,0,"");
    status_write = CAN_Write(h, &msg);

    sleep(1);

    // Read
    DWORD status_read = CAN_Read(h, &msg_r);
    if ((int)msg_r.ID != 1) ROS_ERROR("Force Torque Sensor Initialization: FAILED!");
    if (DEBUG) ATIMini45::printCANMSG(&msg_r);
    std::cout<<"....";
    status_read = CAN_Read(h, &msg_r);
    if ((int)msg_r.ID != 1167) ROS_ERROR("Force Torque Sensor Initialization: FAILED!");
    if (DEBUG) ATIMini45::printCANMSG(&msg_r);
    status_read = CAN_Read(h, &msg_r);
    if ((int)msg_r.ID != 1167) ROS_ERROR("Force Torque Sensor Initialization: FAILED!");
    if (DEBUG) ATIMini45::printCANMSG(&msg_r);
    std::cout<<".... INITIALIZATION OK!";
    ROS_INFO("Force Torque Initialization: SUCCEED!");
    DWORD stt = CAN_Status(h);
  }

  void read()
  {
    ROS_INFO("Read Function");
    TPCANMsg msg;
    DWORD status_read = CAN_Read(h, &msg);
    if (DEBUG) ATIMini45::printCANMSG(&msg);
    if ((int)msg.DATA[0] == 0){
      //now read torques 
    }
    std::cout<<" _______________________\n";
    status_read = CAN_Read(h, &msg);
    if (DEBUG) ATIMini45::printCANMSG(&msg);

  }

  void write()
  {
    ROS_INFO("Write Function");
    TPCANMsg msg;
    ATIMini45::setCANMsg(&msg,0x080,0,"");
    DWORD status_write = CAN_Write(h, &msg);
  }

  // Get Time
  ros::Time get_time() { return ros::Time::now(); };
  // Get Period
  ros::Duration get_period() { return ros::Duration(1.0 / RATE); };
  // Get socket handler 
  HANDLE get_socket_handler(){return h;};
  // Set Device Function
  void set_device(const char* _dev) { _device = _dev;};

private:
  // Force Torque Interface
  ForceTorqueSensorInterface ft_sensor_interface;
  // Force and Torques
  double frc[3];
  double trq[3];
  int indf,indt;
  // Attached Link
  std::string _attached_link = "world";
  // Node Handler
  ros::NodeHandle n_;
  // FD of Socket
  HANDLE h;
  // Device CAN
  const char *_device = "/dev/pcanusb32";
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "FTATIMini45_hw_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(RATE);

  ATIMini45 robot;

  //initialize control manager
  controller_manager::ControllerManager cm(&robot, nh);

  //Initialize Variables
  robot.initCommunication();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration dur = robot.get_period();

  while (ros::ok())
  {
    ros::spinOnce();

    //time loop starts
    ros::Time start = ros::Time::now();

    std::cout << "+++++++++++++++++++++++++++++++++++++++++" << std::endl;
    
    //send data to the robot
    robot.write();

    // update data
    cm.update(start, dur);

    // read robot data
    robot.read();

    //loop_rate.sleep();

    loop_rate.sleep();
    // time loop ends
    dur = robot.get_time() - start;

    //print dt
    printf("Loop dt:%lf\n", dur.toSec());
    std::cout << "+++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }
  DWORD stt = CAN_Close(robot.get_socket_handler());
  return 0;
}
