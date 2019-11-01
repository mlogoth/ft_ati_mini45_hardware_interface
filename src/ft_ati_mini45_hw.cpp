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

bool FD=false;

template< typename T >
std::string int_to_hex( T i )
{
  std::stringstream stream;
  stream << "0x" 
         << std::setfill ('0') << std::setw(sizeof(T)) 
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

  
  void setCANMsg(struct pcanfd_msg *msg, std::string _id, unsigned short _data_len, std::string string ) {
    

    if(_data_len>PCANFD_MAXDATALEN || string.size()>PCANFD_MAXDATALEN){
      ROS_ERROR("Wrong Msg Set");
      exit(-1);
    }


    msg->id = std::stoul(_id, nullptr,16);//0x020f;//std::stoul(_id, nullptr,16);//
    msg->data_len = _data_len;
    msg->flags = PCANFD_MSG_RTR;
    msg->type = PCANFD_TYPE_CAN20_MSG;

    /*

    char mydata[64];
    //mydata=(char*)malloc(64*sizeof(char));
    for (unsigned int elt = 0;elt<65;elt++){
      //*(mydata + elt*sizeof(char)) = '0';
      mydata[elt] = '0';
    }

    for (unsigned int elt = 0;elt<100;elt++){
      std::cout<< mydata[elt]<<std::endl;//*(mydata + elt*sizeof(char)) <<std::endl;
    }
    
    exit(-1);
    */

    //fill data of msg
    //for (unsigned int  i=0;i<string.size();i++) {
    for (unsigned int  i=64-string.size();i<64;i++){
            //msg->data[i]=string[i];
            msg->data[i]=string[i-(64-string.size())];
    }

    //for (unsigned int i=string.size(); i<PCANFD_MAXDATALEN; i++) {
    for (unsigned int i=0; i<64-string.size(); i++) {
        msg->data[i]='0';
    }

    /*
    for(unsigned int i = 0; i<100; i++){
      std::cout<< "i: "<<i <<" | "<< msg->data[i]<<std::endl;
    }
    std::cout<< std::endl;
    */
    std::cout<< "ID: "<< msg->id <<std::endl;
    std::cout<< "Data Length: "<< msg->data_len <<std::endl;
    std::cout<< "Data: " << msg->data <<std::endl;
    
  
  };

  void initCommunication()
  {
    // Opens a CAN 2.0 channel for reading and writing @1Mb bitrate,
    // accepting extended format messages too
    
    if (FD==true) {
        fd = pcanfd_open("/dev/pcanusb32", OFD_BITRATE, 1000000);

        pcanfd_init *pfdinit;
        pfdinit = (pcanfd_init*) malloc(sizeof(pcanfd_init));

        int init_h = pcanfd_get_init(fd, pfdinit);
        
        pcan_bittiming *dt,*nm;
        dt = (pcan_bittiming*) malloc(sizeof(pcan_bittiming));
        nm = (pcan_bittiming*) malloc(sizeof(pcan_bittiming));
        *dt = pfdinit->data;
        *nm = pfdinit->nominal;

        std::cout<< "Clock Hz: "<< pfdinit->clock_Hz << std::endl;
        std::cout<< "Flags: "<< pfdinit->flags << std::endl;
        std::cout<< "Bitrate: " << dt->bitrate << std::endl;
        std::cout<< "Bitrate: " << nm->bitrate << std::endl;
        

        //fd = 1 ;
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
          /*for (pml->count=0; pml->count<6; pml->count++) {
            ATIMini45::setCANMsg(pml->list + pml->count,"0x20f",6, InitData[pml->count]);
          }*/
          
          /* Send Msg
          int err = pcanfd_send_msgs(fd,pml);

          if (err==0){
            ROS_INFO("Write Msg sent correctly!");
          }
          else ROS_ERROR("Unable to Send the MSG");
          */

          std::cout<< "Create MSG" <<std::endl;
          struct pcanfd_msg *msg;
          msg = (struct pcanfd_msg*) malloc(sizeof(struct pcanfd_msg));
          
          msg->type = PCANFD_TYPE_CAN20_MSG;
          
          msg->id = 0x020f;

          msg->data_len = 6;
          msg->data[0] = 0x00;
          msg->data[1] = 0x00;
          msg->data[2] = 0x00;
          msg->data[3] = 0x00;
          msg->data[4] = 0x00;
          msg->data[5] = 0x01;
          std::cout<< "Data: "<<msg->data[0]<<msg->data[1]<<msg->data[2]<<msg->data[3]<<msg->data[4]<<msg->data[5] <<std::endl;
          std::cout<< "Data len: "<<msg->data_len <<std::endl;
          std::cout<< "Data type: "<<msg->data <<std::endl;
          int err = pcanfd_send_msg(fd,msg);

          std::cout<<"err: "<<err<<std::endl;
          if (err==0){
            ROS_INFO("Write Msg sent correctly!");
          }
          else ROS_ERROR("Unable to Send the MSG");
        
        }
    }


    else {
          // CAN Open
          //h = CAN_Open(HW_USB_X6); // Type and Number of port
          // Can initialization with Bitrate 1MBit/sec
          

          HANDLE h = LINUX_CAN_Open("/dev/pcanusb32",02);
          //h = LINUX_CAN_FileHandle(a);

          DWORD status = CAN_Init(h,CAN_BAUD_1M,CAN_INIT_TYPE_EX);
          
          std::cout << "Can INIT Status: "<<status << std::endl;
          
          TPCANMsg msg,msg_r;
          msg.ID = 0x20f;//0x20f;
          msg.LEN = (__u8)6;
          msg.MSGTYPE = MSGTYPE_STANDARD;
          msg.DATA[0] = (__u8)0;
          msg.DATA[1] = (__u8)0;
          msg.DATA[2] = (__u8)0;
          msg.DATA[3] = (__u8)0;
          msg.DATA[4] = (__u8)0;
          msg.DATA[5] = (__u8)1;
          //msg.DATA[6] = (__u8) 0x00;
          //msg.DATA[7] = 0x01;

          std::cout << "____ NEW MESSAGE _____"<<std::endl;
          DWORD status_write = CAN_Write(h,&msg);
          std::cout<< "Write Status: "<<status_write<<std::endl;
          sleep(1);

          msg.ID = 0x20f;//0x20f;
          msg.LEN =(__u8)6;
          msg.MSGTYPE = MSGTYPE_STANDARD;
          msg.DATA[0] = (__u8)0x20;
          msg.DATA[1] = (__u8)0;
          msg.DATA[2] = (__u8)0;
          msg.DATA[3] = (__u8)0;
          msg.DATA[4] = (__u8)0;
          msg.DATA[5] = (__u8)1;
          msg.DATA[6] = (__u8)0;
          //msg.DATA[7] = 0x01;

          std::cout << "____ NEW MESSAGE _____"<<std::endl;
          status_write = CAN_Write(h,&msg);
          std::cout<< "Write Status: "<<status_write<<std::endl;

          sleep(1);

          msg.ID = 0x20f;//0x20f;
          msg.LEN =(__u8)6;
          msg.MSGTYPE = MSGTYPE_STANDARD;
          msg.DATA[0] = (__u8)0x0D;
          msg.DATA[1] = (__u8)0x11;
          msg.DATA[2] = (__u8)0x0D;
          msg.DATA[3] = (__u8)0x40;
          msg.DATA[4] = (__u8)0x0D;
          msg.DATA[5] = (__u8)0x41;
          msg.DATA[6] = (__u8)0x00;



          std::cout << "____ NEW MESSAGE _____"<<std::endl;
          status_write = CAN_Write(h,&msg);
          std::cout<< "Write Status: "<<status_write<<std::endl;

          sleep(1);

          msg.ID = 0x20f;//0x20f;
          msg.LEN =(__u8)6;
          msg.MSGTYPE = MSGTYPE_STANDARD;
          msg.DATA[0] = (__u8)0x0D;
          msg.DATA[1] = (__u8)0x12;
          msg.DATA[2] = (__u8)0x00;
          msg.DATA[3] = (__u8)0x00;
          msg.DATA[4] = (__u8)0x00;
          msg.DATA[5] = (__u8)0x01;
          msg.DATA[6] = (__u8)0x00;

           std::cout << "____ NEW MESSAGE _____"<<std::endl;
          status_write = CAN_Write(h,&msg);
          std::cout<< "Write Status: "<<status_write<<std::endl;
          sleep(1);


          msg.ID = 0x20f;//0x20f;
          msg.LEN =(__u8)6;
          msg.MSGTYPE = MSGTYPE_STANDARD;
          msg.DATA[0] = (__u8)0x2D;
          msg.DATA[1] = (__u8)0x11;
          msg.DATA[2] = (__u8)0x0D;
          msg.DATA[3] = (__u8)0x40;
          msg.DATA[4] = (__u8)0x0D;
          msg.DATA[5] = (__u8)0x41;
          msg.DATA[6] = (__u8)0x00;

           std::cout << "____ NEW MESSAGE _____"<<std::endl;
          status_write = CAN_Write(h,&msg);
          std::cout<< "Write Status: "<<status_write<<std::endl;
          sleep(1);

          msg.ID = 0x20f;//0x20f;
          msg.LEN =(__u8)6;
          msg.MSGTYPE = MSGTYPE_STANDARD;
          msg.DATA[0] = (__u8)0x2D;
          msg.DATA[1] = (__u8)0x12;
          msg.DATA[2] = (__u8)0x00;
          msg.DATA[3] = (__u8)0x00;
          msg.DATA[4] = (__u8)0x00;
          msg.DATA[5] = (__u8)0x01;
          msg.DATA[6] = (__u8)0x00;

           std::cout << "____ NEW MESSAGE _____"<<std::endl;
          status_write = CAN_Write(h,&msg);
          std::cout<< "Write Status: "<<status_write<<std::endl;
          sleep(1);



          int cnt = 1;

          while (cnt<10){
              msg.ID = 0x080;//0x20f;
              msg.LEN = 0x00;
              msg.MSGTYPE = MSGTYPE_STANDARD;
              msg.DATA[0] = (__u8) 0x00;
              msg.DATA[1] = (__u8) 0x00;
              msg.DATA[2] = (__u8) 0x00;
              msg.DATA[3] = (__u8) 0x00;
              msg.DATA[4] = (__u8) 0x00;
              msg.DATA[5] = (__u8) 0x00;
              msg.DATA[6] = (__u8) 0x00;
              //msg.DATA[7] = 0x01;

              std::cout << "____ NEW MESSAGE _____"<<std::endl;
              status_write = CAN_Write(h,&msg);
              std::cout<< "Write Status: "<<status_write<<std::endl;


              // msg.ID = 0x20f;
              // msg.LEN = 0x06;
              // msg.MSGTYPE = MSGTYPE_STANDARD;
              // msg.DATA[0] = 0x2D;
              // msg.DATA[1] = 0x00;
              // msg.DATA[2] = 0x00;
              // msg.DATA[3] = 0x00;
              // msg.DATA[4] = 0x00;
              // msg.DATA[5] = 0x00;
              // msg.DATA[6] = 0x01;
              // status_write = CAN_Write(h,&msg);
              // std::cout<< "Write Status: "<<status_write<<std::endl;
              


              // Sleep for 1 sec
              sleep(1);
              
              // Read
              DWORD status_read = CAN_Read(h,&msg_r);
              std::cout<<"  **  MSG 1 **  \n";
              std::cout<<"Read Status: "<<status_read<<std::endl;
              std::cout<<"MSG Data: \n";
              for(int elt=0;elt<8;elt++) {
                std::cout<< int_to_hex((int) msg_r.DATA[elt])<<" ";
              }//int_to_hex(msg_r.DATA[1])<<" "<<int_to_hex(msg_r.DATA[2])<<std::endl;
              std::cout<<"\nMSG LEN: "<<int_to_hex((int)msg_r.LEN) << " | "<<(int)msg_r.LEN <<std::endl;
              std::cout<<"MSG ID: "<<int_to_hex((int)msg_r.ID)<<" | " << (int)msg_r.ID << std::endl;
              std::cout<<"MSG TYPE: "<<(int)msg_r.MSGTYPE<<std::endl;
              

              status_read = CAN_Read(h,&msg_r);
              std::cout<<"  **  MSG 2 **  \n";
              std::cout<<"Read Status: "<<status_read<<std::endl;
              //std::cout<<"Read MSG Data: "<<int_to_hex(msg_r.DATA[0])<<" "<<int_to_hex(msg_r.DATA[1])<<" "<<int_to_hex(msg_r.DATA[2])<<std::endl;
              std::cout<<"MSG Data: \n";
              for(int elt=0;elt<8;elt++) {
                std::cout<< int_to_hex((int)msg_r.DATA[elt])<<" ";
              }
              std::cout<<"\nMSG LEN: "<<int_to_hex((int)msg_r.LEN)<< " | "<<(int)msg_r.LEN  <<std::endl;
              std::cout<<"MSG ID: "<<int_to_hex((int)msg_r.ID)<<" | " << (int)msg_r.ID << std::endl;
              std::cout<<"MSG TYPE: "<<(int)msg_r.MSGTYPE<<std::endl;

              // Get Status of CAN

              DWORD stt = CAN_Status(h);
              cnt++;
          }
          DWORD stt = CAN_Close(h);
           
          exit(-1);

      }
      

      //struct pcanfd_msg *example_msg;
      /*
      
      
      std::cout<<"MSG LEN: "<<msg.LEN<<std::endl;
      std::cout<<"Write MSG Data: "<<msg.DATA <<std::endl;
      std::cout<<"Read MSG ID: "<<int_to_hex(msg.ID)<<" | " <<msg.ID <<std::endl;
      DWORD status_write = CAN_Write(h,&msg);
      std::cout<< "Write Status: "<<status_write<<std::endl;

      DWORD status_read = CAN_Read(h,&msg_r);
      std::cout<<"Read MSG Data: "<<msg_r.DATA[5] <<std::endl;
      std::cout<<"Read MSG LEN: "<<msg_r.LEN <<std::endl;
      std::cout<<"Read MSG ID: "<<int_to_hex(msg_r.ID)<<" | " << msg_r.ID << std::endl;
      std::cout<<"Read Status: "<<status_read<<std::endl;
      */


      
      


  }

  void read(){ 
    ROS_INFO("Read Function");
  }

  void write(){
    ROS_INFO("Write Function");

    struct pcanfd_msg *msg_to_red;
    msg_to_red = (struct pcanfd_msg*) malloc(sizeof(struct pcanfd_msg));
    
    if (!msg_to_red){
        ROS_ERROR("Memory Allocation Failed Msg to Read");
        exit(-1);
      }
    
    //ATIMini45::setCANMsg(msg_to_red,"0x080",0, "0");

    /*
    int err = pcanfd_send_msg(fd,msg_to_red);

    if (err==0){
      ROS_INFO("Write Msg sent correctly!");
      }
    else ROS_ERROR("Unable to Send the MSG");
    */
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
  HANDLE h;
  // Device CAN
  const char* _device = "/dev/pcanusb32";

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
