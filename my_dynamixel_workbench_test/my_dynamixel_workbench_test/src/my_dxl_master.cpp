/**********************************************************************
//my_dxl_master.cpp
//Date:2020.2.28
//Descripotion:This is a control_master that connect real motor through
//USB,publish motor's state,subscribe motor's control command from motor
//node.
//我们在启动的时候，需要通过配置文件来对舵机进行初始化
//另外需要有一个发布者，发布的内容包括舵机的ID，舵机的电流，位置，速度，力矩等信息
//同时需要一个订阅者，订阅motor节点发布的控制命令，并通过串口发送到实体舵机
**********************************************************************/
#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
#include<my_dynamixel_workbench_test/dxl_state.h>
#include<iostream>
#include <fstream>
#include <sstream>
#include<my_dxl_master.h>
#include<my_dynamixel_workbench_test/ChangeGoalPosition.h>
using namespace std;

#define MOTOR_NUM 3

MyDynamixelController::MyDynamixelController(){
    dxl_wb_ = new DynamixelWorkbench;
    goal_position = 0;
    p_gain = 0.05;//0.1
    i_gain = 0.0;//0.01
    d_gain = 0.05;
}

bool MyDynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(),baud_rate,&log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool MyDynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  //node可以理解为 树的根节点，这里什么时候分叉，需要看缩进符的结构
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;
  //iterator 翻译为 迭代器，有时又称游标（cursor）是程序设计的软件设计模式，可在容器上遍访的接口
  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    //这里觉得是读到用户自定义舵机的别名
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      //把ID号赋值给舵机的别名
      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }
  return true;
}

//按照加载的配置文件，显示当前舵机在线的数量，名字，以及id号
bool MyDynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {      
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}
//初始化舵机，注意写入的时候，舵机的力矩要torqueoff，写入完成，再torque on
bool MyDynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}
//这是初始化控制表，即获得舵机当前值以及目标值
bool MyDynamixelController::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)  return false;

  const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
  if (goal_current == NULL) return false;  

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  control_items_["Goal_Current"] = goal_current;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}
//Handler 事件处理器？ 对象执行某个动作？？
bool MyDynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  //采用通讯协议2.0版本
  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {  
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */    
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}
//同时读取所有舵机的数值，Sync Read 

//同时控制所有舵机的数值，Sync Write

//自定义service callback
void MyDynamixelController::initServer()
{
  changeGoalPositonSrv = node_handle_.advertiseService("changePosition",&MyDynamixelController::changePositionCallback,this);
  changePIDGainSrv = node_handle_.advertiseService("changePidGain",&MyDynamixelController::changePIDGain,this);
}
bool MyDynamixelController::changePositionCallback
    (my_dynamixel_workbench_test::ChangeGoalPosition::Request &req
      ,my_dynamixel_workbench_test::ChangeGoalPosition::Response &res){
    ROS_INFO("request:set goalPosition:%d",req.goal_position);
    setGoalPosition(req.goal_position);
    res.result=true;
    return true;
}
bool MyDynamixelController::changePIDGain
    (my_dynamixel_workbench_test::ChangePIDGain::Request &req
      ,my_dynamixel_workbench_test::ChangePIDGain::Response &res)
{
    ROS_INFO("set pidGain:   p_gain:%f   i_gain:%f   d_gain:%f",req.p_gain,req.i_gain,req.d_gain);
    setPidGain(req.p_gain,req.i_gain,req.d_gain);
    res.result=true;
    return true;
}

//ros timer callback
void MyDynamixelController::writeCallback(const ros::TimerEvent&){

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t dynamixel_position[dynamixel_.size()];//这个是不是应该变为电流？？

  static uint32_t point_cnt = 0;
  static uint32_t position_cnt = 0;


}
void MyDynamixelController::readCallback(const ros::TimerEvent&){

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];//这个什么作用还不清楚，对应的是id的排序 
  uint8_t id_cnt = 0;
  // 遍历 dynamixel_，id_cnt 对应数组的序号，？？如果放在定时器里的话应该是定时读取，有必要吗？
  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  // 输出数据
  for (uint8_t i = 0;i < id_cnt;i++)
  {
      dxl_wb_->itemRead(id_array[i],"Present_Position",&get_position[i]);
    //  ROS_INFO("dynamixel id: %d   present_position:%d",id_array[i],get_position[i]);
    //  dxl_wb_->getRadian(id_array[i],&radian);
      dxl_wb_->itemRead(id_array[i],"Present_Velocity",&get_velocity[i]);//得到的是整数
    //  velocity= dxl_wb_->convertValue2Velocity(ID,velocity_data);  
      dxl_wb_->itemRead(id_array[i],"Present_Current",&get_current[i]);
    //  current = dxl_wb_->convertValue2Current(ID,current_data);
  }

  for(uint8_t index = 0; index < id_cnt; index++)
  {
    dynamixel_state[index].present_current = get_current[index];
    dynamixel_state[index].present_velocity = get_velocity[index];
    dynamixel_state[index].present_position = get_position[index];
    dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
  }

}


// pid controller function
bool MyDynamixelController::getGoalPosition(int & position){
  position = goal_position;
  return true;
}
bool MyDynamixelController::setGoalPosition(int position){
  goal_position = position;
  return true;
}
bool MyDynamixelController::setPidGain(float p,float i,float d){
  p_gain = p;
  i_gain = i;
  d_gain = d;
  return true;
}
bool MyDynamixelController::setLimitCurrent(int lim_cur)
{
  limit_current = lim_cur;
  return true;
}
int MyDynamixelController::pidController(int present_position){
  int goal_current = 0;
  last_position_err = position_err;
  position_err = goal_position - present_position;
  err_integral += position_err;
  //ROS_INFO("p_gain:%f    goal_position=%d,",p_gain,goal_position);

  goal_current = (int) (p_gain * position_err+ i_gain * err_integral +d_gain * (position_err-last_position_err));
  if(goal_current>limit_current)
  {
    goal_current = limit_current;
  }
  if(goal_current<-limit_current)
  {
    goal_current = -limit_current;
  }
  return goal_current;
}

// main function
//void recordDxlState2Txt(my_dynamixel_workbench_test::dxl_state state_msg);
int main(int argc,char ** argv)
{
    string folder="data2.txt";
    std::ofstream fout1(folder.c_str());
  //  fout1<<"hello";
    ros::init(argc,argv,"my_dxl_master");
    ros::NodeHandle node_handle;

    std::string port_name = "/dev/ttyUSB0";
    uint32_t baud_rate = 57600;

    if (argc < 2)
    {
        ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
        return 0;
    }
    else
    {
        port_name = argv[1];
        baud_rate = atoi(argv[2]);
    }

    MyDynamixelController dynamixel_controller;

    bool result = false;

    std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

    result = dynamixel_controller.initWorkbench(port_name, baud_rate);
    if (result == false)
    {
        ROS_ERROR("Please check USB port name");
        return 0;
     }
    
    result = dynamixel_controller.getDynamixelsInfo(yaml_file);
    if (result == false)
    {
        ROS_ERROR("Please check YAML file");
        return 0;
    }

    result = dynamixel_controller.loadDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return 0;
    }

    result = dynamixel_controller.initDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return 0;
    }

    result = dynamixel_controller.initControlItems();
    if (result == false)
    {
        ROS_ERROR("Please check control items");
        return 0;
    }

    result = dynamixel_controller.initSDKHandlers();
    if (result == false)
    {
        ROS_ERROR("Failed to set Dynamixel SDK Handler");
        return 0;
    }

    DynamixelWorkbench * dxl_wb = dynamixel_controller.dxl_wb_;
    //ros topic publish
    ros::Publisher dxl_state_pub = node_handle.advertise<my_dynamixel_workbench_test::dxl_state>("dxl_state_topic",1);
    my_dynamixel_workbench_test::dxl_state state_msg;

    //ros service that you can change goal_position
    dynamixel_controller.initServer();
    //ros::ServiceServer changeGoalPositonSrv = node_handle.advertiseService("changePosition",&changePositionCallback);
    ros::Rate loop_rate = 10;
    //TEST CODE 2020.2.29
    uint8_t ID = 0 ;
    //dxl_wb->ledOn(testID);
    dxl_wb->ledOff(ID);
    ROS_INFO("Welcome my dynamixel workbench!");
    //feedback info
    float radian = 0.0;
    int32_t position_data = 0.0;
    float velocity = 0.0;
    int32_t velocity_data = 0;
    float current = 0.0;
    int32_t current_data = 0;
    //pid current control 注意这里的int字节最好限定字节大小
    int goal_current = 0;
    int present_position = 0;
    int limit_current = 0;

    dxl_wb->setCurrentControlMode(ID);
    dxl_wb->itemRead(ID,"Current_Limit",&limit_current);
    ROS_INFO("Current Limit:%d",limit_current);
    //we have to limit the max&min current  
    limit_current = 50; 
    dynamixel_controller.setLimitCurrent(limit_current);

    ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()),
                                           &MyDynamixelController::readCallback, &dynamixel_controller);
    ros::spin();
/*
    while(ros::ok())
    { 
      //get dxl state include current velocity & radian
      dxl_wb->itemRead(ID,"Present_Position",&present_position);
      dxl_wb->getRadian(ID,&radian);
      dxl_wb->itemRead(ID,"Present_Velocity",&velocity_data);
      velocity= dxl_wb->convertValue2Velocity(ID,velocity_data);  
      dxl_wb->itemRead(ID,"Present_Current",&current_data);
      current = dxl_wb->convertValue2Current(ID,current_data);
      //ROS_INFO("radian:%2.2f degree velocity:%0.3f rpm current:%0.2f mA present_position:%d",radian/3.14*180+180,velocity,current,present_position); 
      //****************************
      //pid control; first step realise p control
      //Expect:goal_position feedback:present_position Input:goal_current output:present_position
      //****************************
      goal_current = dynamixel_controller.pidController(present_position);
      //ROS_INFO("goal_current:%d",goal_current);
      //output current 
      dxl_wb->itemWrite(ID,"Goal_Current",goal_current);
     
      //public topic
      state_msg.id = ID;
      state_msg.present_radian = present_position;
      state_msg.present_velocity = velocity;
      state_msg.present_current = current;
      dxl_state_pub.publish(state_msg);
      //recordDxlState2Txt(state_msg);
      // record data
      fout1 << "stamp:" << ros::Time::now();
      fout1 << "   radian:" << state_msg.present_radian ;
      fout1 << "   velocity:"<< state_msg.present_velocity;
      fout1 << "   current:" << state_msg.present_current <<std::endl;
      //fout1 << std::endl;     
      loop_rate.sleep();
      ros::spinOnce();
    }
    dxl_wb->itemWrite(ID,"Goal_Current",0);
*/
    return 0;
}
/*
void recordDxlState2Txt(my_dynamixel_workbench_test::dxl_state state_msg){
  //fout1 << "stamp:" << state_msg.stamp << std::endl;
  ROS_INFO("record data!");
  fout1 << "radian:" << state_msg.present_radian <<std::endl;
  fout1 << "velocity:"<< state_msg.present_velocity <<std::endl;
  fout1 << "current:" << state_msg.present_current <<std::endl;
  fout1 << std::endl;
}*/