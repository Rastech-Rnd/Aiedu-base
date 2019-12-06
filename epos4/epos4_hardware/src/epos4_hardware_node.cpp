//============================================================================
// Name        : epos4_hardware_node.cpp
// Author      : Jhung JoonYoung
// Version     :
// Copyright   : maxon motor ag 2014-2018
// Description : epos4 driver control in ROS C++
//============================================================================
#include "epos4_hardware/epos4_driver.hpp"
//#include <robot_msgs/ArmJoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <robot_msgs/NanaJoint.h>
#include <math.h>

#define NODE_ID_1   1
#define NODE_ID_2   2
#define NODE_COUNT  2

using namespace std;

CEpos4driver cEpos4driver;


//robot_msgs::ArmJoint armState_msg;
//robot_msgs::ArmJoint armCmd_msg;

ros::Publisher armPos_pub;
ros::Subscriber armCmd_sub;
ros::Subscriber armParam_sub;
ros::Subscriber emergency_sub;
ros::Subscriber epos4Mode_sub;
ros::Subscriber eposPosReset_sub;
ros::Subscriber cmdNanaArm_sub;
ros::Subscriber armJoyVel_sub;

epos4_param motor_param[NODE_COUNT];

int driver_mode = POSITION_MODE;
bool flagThread = false;



/*************************************************************************************************
 * convertRange
 * value: the number to convertRange
 * in_min: the lower bound of the value’s current range
 * in_max: the upper bound of the value’s current range
 * out_min: the lower bound of the value’s target range
 * out_max: the upper bound of the value’s target range
 *************************************************************************************************/
double convertRange(long value, long in_min, long in_max, long out_min, double out_max)
{
  if(value > in_max) value = in_max;
  else if(value < in_min) value = in_min;

  //return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return value * out_max;
}

/*************************************************************************************************
 * callbackEPOS4CmdMsg
 * velocity mode : starts the movement with velocity profile to target velocity.
 * position mode : starts movement with position profile to target position.
 *************************************************************************************************/
void callbackEPOS4CmdMsg(const std_msgs::Int32MultiArray data)
{
  unsigned int ulErrorCode = 0;

  if(driver_mode == VELOCITY_MODE)
  {
    cEpos4driver.SetProfileVelocityMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.SetProfileVelocityMode(NODE_ID_2, ulErrorCode);

    cEpos4driver.MoveProfileVelocity(NODE_ID_1, data.data[NODE_1], ulErrorCode);
    cEpos4driver.MoveProfileVelocity(NODE_ID_2, data.data[NODE_2], ulErrorCode);
  }
  else if(driver_mode == POSITION_MODE)
  {
    int position[NODE_COUNT];
    //position[NODE_1] = convertRange(data.data[NODE_1], ARM_CONTOL_RANGE_MIN, ARM_CONTOL_RANGE_MAX,
    //                                MAXON_EC_MOTOR_100_POS_MIN, MAXON_EC_MOTOR_100_POS_MAX);
    //position[NODE_2] = convertRange(data.data[NODE_2], ARM_CONTOL_RANGE_MIN, ARM_CONTOL_RANGE_MAX,
    //                                MAXON_EC_MOTOR_FUALHABER_POS_MIN, MAXON_EC_MOTOR_FUALHABER_POS_MAX);
    cEpos4driver.SetProfilePositionMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.SetProfilePositionMode(NODE_ID_2, ulErrorCode);

    cEpos4driver.MoveProfilePostion(NODE_ID_1, (int)position[NODE_1], ABSOLUTE, IMMEDIATELY, ulErrorCode);
    cEpos4driver.MoveProfilePostion(NODE_ID_2, (int)position[NODE_2], ABSOLUTE, IMMEDIATELY, ulErrorCode);
  }
}


/*************************************************************************************************
 * callbackEPOS4ParamMsg
 * sets the velocity/position profile parameters
 * ProfileAcceleration : Velocity/Position profile acceleration
 * ProfileDeceleration : Velocity/Position profile acceleration
 * ProfileVelocity : Position profile velocity (only position mode)
 *************************************************************************************************/
void callbackEPOS4ParamMsg(const std_msgs::Int32MultiArray data)
{
  unsigned int ulErrorCode = 0;

  driver_mode = data.data[VELOCITY_MODE];
  if(driver_mode == VELOCITY_MODE)
  {
    cEpos4driver.SetProfileVelocityMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.SetProfileVelocityMode(NODE_ID_2, ulErrorCode);

    cEpos4driver.SetVelocityProfile(NODE_ID_1, motor_param[NODE_1].accelaration,
                                    motor_param[NODE_1].deceleration, ulErrorCode);
    cEpos4driver.SetVelocityProfile(NODE_ID_2, motor_param[NODE_2].accelaration,
                                    motor_param[NODE_2].deceleration, ulErrorCode);
  }
  else if(driver_mode == POSITION_MODE)
  {
    cEpos4driver.SetProfilePositionMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.SetProfilePositionMode(NODE_ID_2, ulErrorCode);

    cEpos4driver.SetPositionProfile(NODE_ID_1, motor_param[NODE_1].velocity, motor_param[NODE_1].accelaration,
                                    motor_param[NODE_1].deceleration, ulErrorCode);
    cEpos4driver.SetPositionProfile(NODE_ID_2, motor_param[NODE_2].velocity, motor_param[NODE_2].accelaration,
                                    motor_param[NODE_2].deceleration, ulErrorCode);
  }
}

/*************************************************************************************************
 * callbackEPOS4EmergencyMsg
 * data = 0 : emergency & pause state off
 * data = 1 : emergency stop
 * data = 2 : pause stop
 *************************************************************************************************/
void callbackEPOS4EmergencyMsg(const std_msgs::Int8 data)
{
  unsigned int ulErrorCode = 0;

  switch (data.data)
  {
  case EMERGENCY_OFF:
    cEpos4driver.g_emergency = EMERGENCY_OFF;
    break;
  case EMERGENCY_STOP:
    cEpos4driver.EmergencyStop(NODE_ID_1, ulErrorCode);
    cEpos4driver.EmergencyStop(NODE_ID_2, ulErrorCode);
    cEpos4driver.g_emergency = EMERGENCY_STOP;
    break;
  case PAUSE_STOP:
    if(driver_mode == VELOCITY_MODE)
    {
      cEpos4driver.HaltVelocityMovement(NODE_ID_1, ulErrorCode);
      cEpos4driver.HaltVelocityMovement(NODE_ID_2, ulErrorCode);
    }
    else if(driver_mode == POSITION_MODE)
    {
      cEpos4driver.HaltPositionMovement(NODE_ID_1, ulErrorCode);
      cEpos4driver.HaltPositionMovement(NODE_ID_2, ulErrorCode);
    }
    cEpos4driver.g_emergency = PAUSE_STOP;
    break;
  default:
    break;
  }
}

/*************************************************************************************************
 * callbackEPOS4ModeMsg
 * data = 0 : changes the operational mode to “profile velocity mode”
 * data = 1 : changes the operational mode to “profile position mode”
 * data = 2 : changes the operational mode to “homing mode”
 *************************************************************************************************/
void callbackEPOS4ModeMsg(const std_msgs::Int8 data)
{
  unsigned int ulErrorCode = 0;

  switch (data.data)
  {
  case VELOCITY_MODE:
    cEpos4driver.SetProfileVelocityMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.SetProfileVelocityMode(NODE_ID_2, ulErrorCode);
    driver_mode = VELOCITY_MODE;
    break;
  case POSITION_MODE:
    cEpos4driver.SetProfilePositionMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.SetProfilePositionMode(NODE_ID_2, ulErrorCode);
    driver_mode = POSITION_MODE;
    break;
  case HOMING_MODE:
    cEpos4driver.SetProfileHomingMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.SetProfileHomingMode(NODE_ID_2, ulErrorCode);
    driver_mode = HOMING_MODE;
    break;
  default:
    break;
  }
}

/*************************************************************************************************
 * callbackEPOS4PosResetMsg
 * true = set a new home position
 *************************************************************************************************/
void callbackEPOS4PosResetMsg(const std_msgs::Int8 data)
{
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;

  if(data.data == 0)
  {
    //cEpos4driver.SetProfileHomingMode(NODE_ID_1, ulErrorCode);
    //cEpos4driver.SetProfileHomingMode(NODE_ID_2, ulErrorCode);

    cEpos4driver.DefinePosition(NODE_ID_1, HOME_POSITION, ulErrorCode);
    cEpos4driver.DefinePosition(NODE_ID_2, HOME_POSITION, ulErrorCode);

    std_msgs::Int8  data;
    data.data = driver_mode;
    callbackEPOS4ModeMsg(data);
  }
}

void callbackNanaArmCmdMsg(const robot_msgs::NanaJoint data)
{
  unsigned int ulErrorCode = 0;

  int position[NODE_COUNT];
  position[NODE_2] = (int)convertRange(-data.position[0], ARM_CONTOL_RANGE_MIN, ARM_CONTOL_RANGE_MAX,
                                  0, 1.85);//s p 1.568888889
  position[NODE_1] = (int)convertRange(-data.position[1], ARM_CONTOL_RANGE_MIN, ARM_CONTOL_RANGE_MAX,
                                  0, 1.333333333);// s y

  cEpos4driver.MoveProfilePostion(NODE_ID_1, (int)position[NODE_1], ABSOLUTE, IMMEDIATELY, ulErrorCode);
  cEpos4driver.MoveProfilePostion(NODE_ID_2, (int)position[NODE_2], ABSOLUTE, IMMEDIATELY, ulErrorCode);
}

void callbackArmJoyVelMsg(const robot_msgs::NanaJoint& msg)
{
  int nodeID;
  int vel;
  unsigned int ulErrorCode = 0;

  nodeID = msg.position[0];
  vel = msg.position[1];
  vel = (int)convertRange(vel, -100, 100, 0, 1);
  vel = vel * 4;

  if(nodeID == 31)
  {
    cEpos4driver.SetProfileVelocityMode(NODE_ID_2, ulErrorCode);
    cEpos4driver.MoveProfileVelocity(NODE_ID_2, vel, ulErrorCode);
  }
  else if(nodeID == 32)
  {
    cEpos4driver.SetProfileVelocityMode(NODE_ID_1, ulErrorCode);
    cEpos4driver.MoveProfileVelocity(NODE_ID_1, vel, ulErrorCode);
  }
}

void threadGetActualPosition(int* rate)
{
//  int n;
//  std::stringstream ss;
//  char buffer[256];
//  std_msgs::String message;
//  ros::Rate loop_rate(1);

//  while (flagThread)
//  {
//    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
//    //ros::Subscriber compressedimage_sub = node->subscribe("/image_raw/compressed", 100, msgCallback_compressedimage);  //Compressed image
//    ros::Subscriber compressedimage_sub = node->subscribe("camera/color/image_raw/compressed", 1, msgCallback_compressedimage);  //Compressed image
//    ros::spin();
//    //loop_rate.sleep();
//  }
}

void sendActualPosition()
{
  int ActualPos[NODE_COUNT]={};
  std_msgs::Int32MultiArray pos;
  unsigned int ulErrorCode = 0;

  cEpos4driver.GetActualPosition(NODE_ID_1, ActualPos[NODE_1], ulErrorCode);
  cEpos4driver.GetActualPosition(NODE_ID_2, ActualPos[NODE_2], ulErrorCode);

  pos.data.push_back(ActualPos[NODE_1]);
  pos.data.push_back(ActualPos[NODE_2]);

  armPos_pub.publish(pos);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "epos4_driver");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  ros::Duration d(10); // 0.01 = 100Hz   0.1 = 10Hz   1 = 1Hz
  ros::Rate r(100);

  for(int i=0; i<NODE_COUNT;i++)
  {
    motor_param[i].mode = EPOS4_MODE_VELOCITY;
    motor_param[i].accelaration = EPOS4_PARAM_ACCELARATION;
    motor_param[i].deceleration = EPOS4_PARAM_DECELERATION;
    if(motor_param[i].mode == EPOS4_MODE_POSITION) motor_param[i].velocity = EPOS4_MODE_VELOCITY;
    else motor_param[i].velocity = 0;
  }

  //private_nh.getParam("mode", motor_param[NODE_1].mode);
  //private_nh.getParam("accelatation", motor_param[NODE_1].accelaration);
  //private_nh.getParam("deceleration", motor_param[NODE_1].deceleration);
  //private_nh.getParam("velocity", motor_param[NODE_1].velocity);

  motor_param[NODE_2].mode = motor_param[NODE_1].mode;
  motor_param[NODE_2].accelaration = motor_param[NODE_1].accelaration;
  motor_param[NODE_2].deceleration = motor_param[NODE_1].deceleration;
  motor_param[NODE_2].velocity = motor_param[NODE_1].velocity;

  /****************************************
   * 초기화 할것들
   * 모터 연결
   * 모터 파라메타 세팅
   * 모터 위치 초기화
   ****************************************/

  armPos_pub = nh.advertise<std_msgs::Int32MultiArray>("/epos4/pos/status", 10);
  armCmd_sub = nh.subscribe("/epos4/cmd", 10, callbackEPOS4CmdMsg);
  armParam_sub = nh.subscribe("/epos4/param", 10, callbackEPOS4ParamMsg);
  emergency_sub = nh.subscribe("/epos4/emergency", 10, callbackEPOS4EmergencyMsg);
  epos4Mode_sub = nh.subscribe("/epos4/mode", 10, callbackEPOS4ModeMsg);
  eposPosReset_sub = nh.subscribe("/nana/pos/reset", 10, callbackEPOS4PosResetMsg);
  cmdNanaArm_sub = nh.subscribe("/maxon/arm/command", 10, callbackNanaArmCmdMsg);
  armJoyVel_sub = nh.subscribe("/nana/arm/vel", 10, callbackArmJoyVelMsg);
  
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;

  cEpos4driver.SetDefaultParameters();
  
  cEpos4driver.OpenDevice(&ulErrorCode);

  cEpos4driver.EnableDevice(NODE_ID_1, &ulErrorCode);
  cEpos4driver.EnableDevice(NODE_ID_2, &ulErrorCode);
  cEpos4driver.SetProfilePositionMode(NODE_ID_1, ulErrorCode);
  cEpos4driver.SetProfilePositionMode(NODE_ID_2, ulErrorCode);

//  cEpos4driver.SetProfileVelocityMode(NODE_ID_1, ulErrorCode);
//  cEpos4driver.SetProfileVelocityMode(NODE_ID_2, ulErrorCode);
//  cEpos4driver.MoveProfileVelocity(NODE_ID_1, 500, ulErrorCode);
//  cEpos4driver.MoveProfileVelocity(NODE_ID_2, 500, ulErrorCode);
//  sleep(5);
//  cEpos4driver.MoveProfileVelocity(NODE_ID_1, 0, ulErrorCode);
//  cEpos4driver.MoveProfileVelocity(NODE_ID_2, 0, ulErrorCode);

  cEpos4driver.SetVelocityProfile(NODE_ID_1, motor_param[NODE_1].accelaration,
                                  motor_param[NODE_1].deceleration, ulErrorCode);
  cEpos4driver.SetVelocityProfile(NODE_ID_2, motor_param[NODE_2].accelaration,
                                  motor_param[NODE_2].deceleration, ulErrorCode);

  cEpos4driver.SetPositionProfile(NODE_ID_1, motor_param[NODE_1].velocity, motor_param[NODE_1].accelaration,
                                  motor_param[NODE_1].deceleration, ulErrorCode);
  cEpos4driver.SetPositionProfile(NODE_ID_2, motor_param[NODE_2].velocity, motor_param[NODE_2].accelaration,
                                  motor_param[NODE_2].deceleration, ulErrorCode);

  while(ros::ok())
  {
    sendActualPosition();
    //d.sleep();
    ros::spinOnce();
    usleep(1000);
  }

  ros::shutdown();

  return lResult;
}

