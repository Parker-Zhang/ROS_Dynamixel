#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include<my_dynamixel_workbench_test/dxl_state.h>
#include<my_dynamixel_workbench_test/ChangeGoalPosition.h>
#include<my_dynamixel_workbench_test/ChangePIDGain.h>
#include <yaml-cpp/yaml.h>
using namespace std;

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT  2


typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class MyDynamixelController
{
private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;      

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;

  // ROS Service Server
  ros::ServiceServer changeGoalPositonSrv;
  ros::ServiceServer changePIDGainSrv;

  // timer period
  double write_period_ = 0.01;
  double read_period_ = 0.005;

  // pid controller parameters
  int goal_position = 0;
  int position_err = 0;
  int last_position_err = 0;
  int err_integral = 0;  
  float p_gain = 0.05;
  float i_gain = 0;
  float d_gain = 0.05;
  int limit_current = 50;

  
public:
  // Dynamixel Workbench Parameters
    DynamixelWorkbench *dxl_wb_;
    MyDynamixelController();
    bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
    bool getDynamixelsInfo(const std::string yaml_file);
    bool loadDynamixels(void);
    bool initDynamixels(void);
    bool initControlItems(void);
    bool initSDKHandlers(void);
    void initServer();

  // callback
    bool changePositionCallback(my_dynamixel_workbench_test::ChangeGoalPosition::Request &req
                                  ,my_dynamixel_workbench_test::ChangeGoalPosition::Response &res);
    bool changePIDGain(my_dynamixel_workbench_test::ChangePIDGain::Request &req
                          ,my_dynamixel_workbench_test::ChangePIDGain::Response &res);                              
  // ros timer callback 
    double getReadPeriod(){return read_period_;}
    double getWritePeriod(){return write_period_;}
    void writeCallback(const ros::TimerEvent&);
    void readCallback(const ros::TimerEvent&);
  
  // pid controller function
    bool setGoalPosition(int position);
    bool getGoalPosition(int &position);
    bool setPidGain(float p_gain,float i_gain,float d_gain);
    bool setLimitCurrent(int lim_cur);
    int pidController(int current_position);
};