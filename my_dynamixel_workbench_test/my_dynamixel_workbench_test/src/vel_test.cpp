#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
#include<my_dynamixel_workbench_test/dxl_state.h>

#include<iostream>
#include <fstream>
#include <sstream>
using namespace std;

#define BAUDRATE 57600
#define ID 0

string folder="data1.txt";
std::ofstream fout1(folder.c_str());

//functions
void recordDxlState2Txt(my_dynamixel_workbench_test::dxl_state state_msg);


int main(int argc,char **argv)
{
  ros::init(argc,argv,"my_dxl_node");
  ros::NodeHandle n;
  ros::Publisher dxl_state_pub = n.advertise<my_dynamixel_workbench_test::dxl_state>("dxl_state_topic",100);
  //I have defined a new msg
  my_dynamixel_workbench_test::dxl_state msg;
  
  //file test
  //fout1.open("~/data1.txt");
  //sleep(1);
  //fout1 << "current velocity  radian";
 // sleep(1);
  //fout1.close();

  DynamixelWorkbench dxl_wb; 
  dxl_wb.begin("/dev/ttyUSB0",BAUDRATE);
  dxl_wb.ping(ID);

  dxl_wb.ledOff(ID);
//  dxl_wb.torqueOff(ID);
//  dxl_wb.setVelocityBasedProfile(ID);
  dxl_wb.torqueOn(ID);
  ROS_INFO("Welcome my dynamixel workbench!");

  //feedback info
  float radian = 0.0;
  int32_t position_data = 0.0;
  float velocity = 0.0;
  int32_t velocity_data = 0;
  float current = 0.0;
  int32_t current_data = 0;

  //dxl_wb.setPositionControlMode(ID);
  dxl_wb.setCurrentControlMode(ID);
  int count = 0;
  int symbol = -1;

  //pid current control 
  int goal_position = 2000;
  int goal_current = 0;
  int position_err = 0;
  int last_position_err = 0;
  int err_integral = 0;
  float p_gain = 0.1;//0.1
  float i_gain = 0.0;//0.01
  float d_gain = 0;
  int present_position = 0;
  int limit_current = 0;

  dxl_wb.itemRead(ID,"Current_Limit",&limit_current);
  ROS_INFO("Current Limit:%d",limit_current);
  //we have to limit the max&min current  
  limit_current = 50;  

  //msg
  ros::Rate loop_rate(10);
  msg.id = ID;

  while(ros::ok())
  {
   count++;
   if(count==20)
   {
     count = 0;
     symbol = -symbol;
    // dxl_wb.goalPosition(ID,2000+symbol*200);
   // fout1.close();
    // return 0;
   }

   //get dxl state include current velocity & radian
   dxl_wb.itemRead(ID,"Present_Position",&present_position);
   dxl_wb.getRadian(ID,&radian);
   dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
   velocity= dxl_wb.convertValue2Velocity(ID,velocity_data);  
   dxl_wb.itemRead(ID,"Present_Current",&current_data);
   current = dxl_wb.convertValue2Current(ID,current_data);
   ROS_INFO("radian:%2.2f degree velocity:%0.3f rpm current:%0.2f mA present_position:%d",radian/3.14*180+180,velocity,current,present_position); 
   /****************************/
   //pid control; first step realise p control
   //Expect:goal_position feedback:present_position Input:goal_current output:present_position
   /****************************/
   last_position_err = position_err;
   position_err = goal_position - present_position;
   err_integral += position_err;
   goal_current = (int) (p_gain * position_err+ i_gain * err_integral +d_gain * (position_err-last_position_err));
   if(goal_current>limit_current)
   {
      goal_current = limit_current;
   }
   if(goal_current<-limit_current)
   {
      goal_current = -limit_current;
   }
   ROS_INFO("goal_current:%d",goal_current);

   dxl_wb.itemWrite(ID,"Goal_Current",goal_current);
   //output current 

   //publish message & record dxl_state data
   //msg.stamp = ros::Time::now();
   //msg.present_radian = radian;
   msg.present_radian = present_position;
   msg.present_velocity = velocity;
   msg.present_current = current;
   dxl_state_pub.publish(msg); 
   recordDxlState2Txt(msg);
   loop_rate.sleep();
  }
  return 0;
}

void recordDxlState2Txt(my_dynamixel_workbench_test::dxl_state state_msg){
  
  //fout1 << "stamp:" << state_msg.stamp << std::endl;
  fout1 << "radian:" << state_msg.present_radian <<std::endl;
  fout1 << "velocity:"<< state_msg.present_velocity <<std::endl;
  fout1 << "current:" << state_msg.present_current <<std::endl;
  fout1 << std::endl;
}
