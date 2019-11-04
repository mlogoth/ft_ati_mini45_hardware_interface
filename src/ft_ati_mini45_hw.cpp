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

bool DEBUG = false;

template <typename T>
std::string int_to_hex(T i)
{
  std::stringstream stream;
  stream << "0x"
         << std::setfill('0') << std::setw(sizeof(T))
         << std::hex << i;
  return stream.str();
}

#define RATE 100.0

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

  void printCANMSG(TPCANMsg *_msg)
  {
    std::cout << " | Data: \n | ";
    for (int elt = 0; elt < 8; elt++)
    {
      std::cout << int_to_hex((int)_msg->DATA[elt]) << " ";
    }
    std::cout << "\n | LENGTH: " << int_to_hex((int)_msg->LEN) << " - " << (int)_msg->LEN << std::endl;
    std::cout << " | ID: " << int_to_hex((int)_msg->ID) << " - " << (int)_msg->ID << std::endl;
    std::cout << " | TYPE: " << (int)_msg->MSGTYPE << std::endl;
  }

  // 2's Complement conversion
  std::string Compl2s(std::string _data)
  {
    for (unsigned int i = 0; i < _data.size(); i++)
    {
      if (_data[i] == '0')
      {
         _data[i] = '1';
      }
      else
      {
        _data[i] = '0';
      }
    }
    return _data;
  }

  // Binary to decimal Number
  int BinToDec(long long n)
  {
    int decimalNumber = 0, i = 0, remainder;
    while (n != 0)
    {
      remainder = n % 10;
      n /= 10;
      decimalNumber += remainder * pow(2, i);
      ++i;
    }
    return decimalNumber;
  }

  // function to convert
  // Hexadecimal to Binary Number
  std::string HexToBin(int hexdec)
  {
        int i=0;
        std::string res;     
        for(i=0; hexdec>0; i++)    
        {    
            if(i==0){
              res.push_back(hexdec%2+'0');
            }
            else{res.insert(0,1,hexdec%2+'0');}    
            hexdec= hexdec/2;
            //std::cout<<i<<std::endl;
            //i++;
        }     
        for(i=i-1 ;i<7 ;i++)    
        {    
            res.insert(0,1,'0');
        }
        //std::cout<<"BINARY: "<<res<<std::endl;
        return res; 

  }    

  
  void setCANMsg(TPCANMsg *_msg, int _id, int _data_len, std::string _data)
  {
    if (_data_len > 8 || _data.size() / 2 != _data_len)
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
    for (unsigned int i = 0; i < _data.size(); i += 2)
    {
      _msg->DATA[cnt] = (__u8)std::stoul(_data.substr(i, 2), nullptr, 16);
      cnt++;
    }
    // fill with zero values the rest of DATA
    while (cnt < 8)
    {
      _msg->DATA[cnt] = (__u8)0x00;
      cnt++;
    }

    if (DEBUG)
      ATIMini45::printCANMSG(_msg);
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
    ATIMini45::setCANMsg(&msg, 0x20f, 6, "000000000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;

    std::cout << " ---- 2nd INIT MSG  ----" << std::endl;
    ATIMini45::setCANMsg(&msg, 0x20f, 6, "200000000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;

    std::cout << " ---- 3rd INIT MSG  ----" << std::endl;
    ATIMini45::setCANMsg(&msg, 0x20f, 6, "0D110D400D41");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;

    std::cout << " ---- 4th INIT MSG  ---- " << std::endl;
    ATIMini45::setCANMsg(&msg, 0x20f, 6, "0D1200000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;

    std::cout << " ---- 5th INIT MSG  ---- " << std::endl;
    ATIMini45::setCANMsg(&msg, 0x20f, 6, "2D110D400D41");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;

    std::cout << " ---- 6th INIT MSG  ---- " << std::endl;
    ATIMini45::setCANMsg(&msg, 0x20f, 6, "2D1200000001");
    status_write = CAN_Write(h, &msg);
    //std::cout << "Write Status: " << status_write << std::endl;

    std::cout << "....";
    ATIMini45::setCANMsg(&msg, 0x080, 0, "");
    status_write = CAN_Write(h, &msg);

    sleep(1);

    // Read
    DWORD status_read = CAN_Read(h, &msg_r);
    if ((int)msg_r.ID != 1)
      ROS_ERROR("Force Torque Sensor Initialization: FAILED!");
    if (DEBUG)
      ATIMini45::printCANMSG(&msg_r);
    std::cout << "....";
    status_read = CAN_Read(h, &msg_r);
    if ((int)msg_r.ID != 1167)
      ROS_ERROR("Force Torque Sensor Initialization: FAILED!");
    if (DEBUG)
      ATIMini45::printCANMSG(&msg_r);
    status_read = CAN_Read(h, &msg_r);
    if ((int)msg_r.ID != 1167)
      ROS_ERROR("Force Torque Sensor Initialization: FAILED!");
    if (DEBUG)
      ATIMini45::printCANMSG(&msg_r);
    std::cout << ".... INITIALIZATION OK!";
    ROS_INFO("Force Torque Initialization: SUCCEED!");
    DWORD stt = CAN_Status(h);
  }

  void read()
  {
    ROS_INFO("Read Function");
    TPCANMsg msg[2];
    DWORD status_read = CAN_Read(h, &msg[0]);
    if (DEBUG)
      ATIMini45::printCANMSG(&msg[0]);
    status_read = CAN_Read(h, &msg[1]);
    if (DEBUG)
      ATIMini45::printCANMSG(&msg[1]);

    if ((int)msg[0].DATA[7] != (int)msg[1].DATA[7])
    {
      if(DEBUG){
        std::cout<<"MS1: "<<(int)msg[0].DATA[7]<<std::endl;
        std::cout<<"MS2: "<<(int)msg[1].DATA[7]<<std::endl;
      }
      ROS_ERROR("DATA Messages Not Synchronized!");
    }

    for (unsigned int i = 0; i < 2; i++)
    {
      if ((int)msg[i].DATA[0] == 0)
      {
        // TORQUES 
        /*
        msg[i].DATA[1]=0x02;
        msg[i].DATA[2]=0xd2;
        msg[i].DATA[3]=0xfd;
        msg[i].DATA[4]=0x4a;
        msg[i].DATA[5]=0x03;
        msg[i].DATA[6]= 0x00;
        */

        
        
        float data[3];
        unsigned int cnt = 0;
        //std::cout<<"-------------------\nDATA: ";
        //for(unsigned int ff=1;ff<7;ff++){std::cout<<int_to_hex((int)msg[i].DATA[ff])<<"|";}
        //std::cout<<std::endl;
        for (unsigned int elt = 1; elt < 6; elt += 2)
        {
          std::string binary;
          double sign=1.0;
          //std::cout<<"Data ["<<elt<<"]: "<<(int)msg[i].DATA[elt] <<"| Data ["<<elt+1<<"]: "<<(int)msg[i].DATA[elt+1]<<std::endl;
          binary = HexToBin((int)msg[i].DATA[elt]) + HexToBin((int)msg[i].DATA[elt + 1]);// + HexToBin((int)msg[i].DATA[elt + 2]) + HexToBin((int)msg[i].DATA[elt + 3]);
          //std::cout<<"Binary: "<<binary<<std::endl;
          //check the MSB
          if (binary[0] == '1')
          {
            binary = Compl2s(binary);
            sign=-1.0;
          }

          //std::cout<<"Binary["<<cnt<<"] :"<< int_to_hex((int)std::stoull(binary, NULL, 2)) <<std::endl;
          double dec = sign*(double)std::stoull(binary, NULL, 2);
          //std::cout<<"Dec: "<<dec<<std::endl;
          data[cnt] = dec;
          cnt++;
        }
        trq[0] = (double)data[1]/100.0;
        trq[1] = (double)data[0]/100.0;
        frc[2] = (double)data[2]/10.0;
        
        // Index Calculation
        std::string binary = HexToBin((int)msg[i].DATA[7]);
        indt = std::stoull(binary, NULL, 2);

      }
    
      else if ((int)msg[i].DATA[0] == 1)
      {
        /*
        msg[i].DATA[1]=0xe1;
        msg[i].DATA[2]=0x7b;
        msg[i].DATA[3]=0xce;
        msg[i].DATA[4]=0x42;
        msg[i].DATA[5]=0x03;
        msg[i].DATA[6]= 0xeb;
        */
        // FORCES
        float data[3];
        unsigned int cnt = 0;
        //std::cout<<"-------------------\nDATA: ";
        //for(unsigned int ff=1;ff<7;ff++){std::cout<<int_to_hex((int)msg[i].DATA[ff])<<"|";}
        //std::cout<<std::endl;
        for (unsigned int elt = 1; elt < 6; elt += 2)
        {
          std::string binary;
          int sign=1;
          //std::cout<<"Data ["<<elt<<"]: "<<(int)msg[i].DATA[elt] <<"| Data ["<<elt+1<<"]: "<<(int)msg[i].DATA[elt+1]<<std::endl;
          binary = HexToBin((int)msg[i].DATA[elt]) + HexToBin((int)msg[i].DATA[elt + 1]);// + HexToBin((int)msg[i].DATA[elt + 2]) + HexToBin((int)msg[i].DATA[elt + 3]);
          //std::cout<<"Binary: "<<binary<<std::endl;
          //check the MSB
          if (binary[0] == '1')
          {
            binary = Compl2s(binary);
            
            sign=-1.0;
          }
          //std::cout<<"Binary["<<cnt<<"] :"<< int_to_hex((int)std::stoull(binary, NULL, 2)) <<std::endl;
          //std::cout<<"Binary: "<<binary<<std::endl;
          double dec = sign*(double)std::stoull(binary, NULL, 2);
          //std::cout<<"Dec: "<<dec<<std::endl;
          data[cnt] = dec;
          cnt++;
        }
        frc[0] = (double)data[1]/10.0;
        frc[1] = (double)data[0]/10.0;
        trq[2] = (double)data[2]/100.0;
        
        // Index Calculation
        std::string binary = HexToBin((int)msg[i].DATA[7]);
        indf = std::stoull(binary, NULL, 2);
      }
      else 
        {
          std::cout<<"MS1: "<<(int)msg[0].DATA[0]<<std::endl;
          std::cout<<"MS2: "<<(int)msg[1].DATA[0]<<std::endl;
          ROS_ERROR("Not well Defined Message Read!");
        }
      
    }
          std::cout<<"Forces: \n"<<"x: "<<frc[0]<<" |y: "<<frc[1]<<" |z: "<<frc[2]<<std::endl;
          std::cout<<"Torques: \n"<<"x: "<<trq[0]<<" |y: "<<trq[1]<<" |z: "<<trq[2]<<std::endl;
  }

void
write()
{
  ROS_INFO("Write Function");
  TPCANMsg msg;
  ATIMini45::setCANMsg(&msg, 0x080, 0, "");
  DWORD status_write = CAN_Write(h, &msg);
}

// Get Time
ros::Time get_time() { return ros::Time::now(); };
// Get Period
ros::Duration get_period() { return ros::Duration(1.0 / RATE); };
// Get socket handler
HANDLE get_socket_handler() { return h; };
// Set Device Function
void set_device(const char *_dev) { _device = _dev; };

private:
  // Force Torque Interface
  ForceTorqueSensorInterface ft_sensor_interface;
  // Force and Torques
  double frc[3];
  double trq[3];
  int indf, indt;
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
