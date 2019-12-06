//============================================================================
// Name        : epos4_hardware_node.cpp
// Author      : Jhung JoonYoung
// Version     : 1.0
// Copyright   : maxon motor ag 2014-2018
// Description : epos4 driver control in ROS C++
//============================================================================
#include "epos4_hardware/epos4_driver.hpp"
//#include "epos4_hardware/CLog.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <vector>


using namespace std;


/*************************************************************************************************
 * Define Motor position
 *************************************************************************************************/
#define NODE_ID_LEFT    1
#define NODE_ID_RIGHT   2
#define NODE_CONUT      2

/*************************************************************************************************
 * Define Motor param
 *************************************************************************************************/
#define GEAR_RATE                 51   // 51:1
#define DRIVER_ACCELARATION       6000
#define DRIVER_DECELERATION       6000

#define WHEEL_RADIUS              0.0675 // m
#define WHEEL_SEPARATION          0.28   // m
#define WHEEL_NUM                 2

#define VELOCITY_CONSTANT_VALUE   156.148708285 // V = r * w = r      *       (RPM              * ( 2 * 3.14 / 60))
                                                //           = 0.0675 * (0.906 * Goal_Velocity) * 0.10472)
                                                // Goal_Velocity = V  * 156.148708285

#define LIMIT_MAX_VELOCITY        161   //(100ms, ppr, 2ch) // MAX RPM is 146 when powered 12.0v
                                        // 146 / 0.906 (RPM) = 161.147...

#define ERROR_ON  true;
#define ERROR_OFF false;

CEpos4driver  cEpos4driver;
//CLog          cLog;

ros::Publisher  motor_vel_pub;
ros::Publisher  error_pub;
ros::Subscriber joy_cmd_sub;
ros::Subscriber motor_param_sub;
ros::Subscriber emergency_sub;

epos4_param motor_param[NODE_CONUT];

int     driver_mode = POSITION_MODE;
bool    flagThread = false;


/*************************************************************************************************
 * convertRange
 * value: the number to convertRange
 * in_min: the lower bound of the value’s current range
 * in_max: the upper bound of the value’s current range
 * out_min: the lower bound of the value’s target range
 * out_max: the upper bound of the value’s target range
 *************************************************************************************************/
double convertRange(double value, double in_min, double in_max, double out_min, double out_max)
{
  if(value > in_max) value = in_max;
  else if(value < in_min) value = in_min;

  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//  return value * out_max;
}

/*******************************************************************************
* Calculation function
*******************************************************************************/
double constrain(double value, double min, double max)
{
  if(value <= min) value = min;
  else  if(value >= max) value = max;

  return value;
}

/*************************************************************************************************
 * callbackJoyCmdMsg
 * get joystic commend and send motor speed
 *************************************************************************************************/
void callbackJoyCmdMsg(const geometry_msgs::Twist& msg)
{
    string  str;
    stringstream  ss;
    unsigned int ulErrorCode = 0;

    double  wheel_left;
    double  wheel_right;

//    wheel_left  = msg.linear.x + (msg.angular.z * WHEEL_SEPARATION / 2);   <<
//    wheel_right = msg.linear.x - (msg.angular.z * WHEEL_SEPARATION / 2);   <<
    wheel_left  = msg.linear.x + (msg.angular.z * WHEEL_SEPARATION * 1.4);
    wheel_right = msg.linear.x - (msg.angular.z * WHEEL_SEPARATION * 1.4);
    //  wheel_left  =  constrain(wheel_left * VELOCITY_CONSTANT_VALUE,  -LIMIT_MAX_VELOCITY, LIMIT_MAX_VELOCITY);
    //  wheel_right =  constrain(wheel_right * VELOCITY_CONSTANT_VALUE, -LIMIT_MAX_VELOCITY, LIMIT_MAX_VELOCITY);


  //  ROS_INFO("LETF : %f", wheel_left);
  //  ROS_INFO("RIGHT : %f", wheel_right);
    wheel_left  =  convertRange(wheel_left,   -1, 1,  -3000, 3000);
    wheel_right =  convertRange(wheel_right,  -1, 1,  -3000, 3000);

    wheel_left  =  constrain(wheel_left,  -3000, 3000) * -1;
    wheel_right =  constrain(wheel_right, -3000, 3000);

    cEpos4driver.MoveProfileVelocity(NODE_ID_LEFT,  wheel_left,   ulErrorCode);
    cEpos4driver.MoveProfileVelocity(NODE_ID_RIGHT, wheel_right,  ulErrorCode);

    ss << "epos4_bardware_2wheel_node::wheel left vel : " << wheel_left << endl;
    str = ss.str();
  //  cLog.sendLogMsg(cLog.Info, str);
    ss.clear();
    ss << "epos4_bardware_2wheel_node::wheel right vel : " << wheel_right << endl;
    str = ss.str();
  //  cLog.sendLogMsg(cLog.Info, str);
}

/*************************************************************************************************
 * callbackMotorParamMsg
 * data[0] : data = 0 -> all , 1~4 -> node number
 * data[1] : parameter accelaration
 * data[2] : parameter deceleration
 *************************************************************************************************/
void callbackMotorParamMsg(const std_msgs::Int32MultiArray& msg)
{
  string str;
  stringstream ss;
  unsigned int ulErrorCode = 0;

  if(msg.data[0] == 0)
  {
    for(int i=0; i<NODE_CONUT; i++)
    {
      cEpos4driver.SetVelocityProfile(i+1, msg.data[1], msg.data[2], ulErrorCode);
      ss << "epos4_bardware_2wheel_node::set node" << i+1 << ", acc:" << msg.data[1] << ", dec:" << msg.data[2] << endl;
      str = ss.str();
//      cLog.sendLogMsg(cLog.Info, str);
      ss.clear();
    }
  }
  else
  {
    cEpos4driver.SetVelocityProfile(msg.data[0], msg.data[1], msg.data[2], ulErrorCode);
    ss << "epos4_bardware_2wheel_node::set node" << msg.data[0] << ", acc:" << msg.data[1] << ", dec:" << msg.data[2] << endl;
    str = ss.str();
//    cLog.sendLogMsg(cLog.Info, str);
  }
}

/*************************************************************************************************
 * callbackEPOS4EmergencyMsg
 * data = 0 : emergency & pause state off
 * data = 1 : emergency stop
 * data = 2 : pause stop
 *************************************************************************************************/
void callbackEmergencyMsg(const std_msgs::Int8 data)
{
  string str;
  stringstream ss;
  unsigned int ulErrorCode = 0;
  switch (data.data)
  {
  case EMERGENCY_OFF:
    cEpos4driver.g_emergency = EMERGENCY_OFF;
    ss << "epos4_bardware_2wheel_node::emergency/pause off" << endl;
    str = ss.str();
//    cLog.sendLogMsg(cLog.Info, str);
    break;
  case EMERGENCY_STOP:
    cEpos4driver.EmergencyStop(NODE_ID_LEFT, ulErrorCode);
    cEpos4driver.EmergencyStop(NODE_ID_RIGHT, ulErrorCode);
    cEpos4driver.g_emergency = EMERGENCY_STOP;
    ss << "epos4_bardware_2wheel_node::emergency stop" << endl;
    str = ss.str();
//    cLog.sendLogMsg(cLog.Error, str);
    break;
  case PAUSE_STOP:
    cEpos4driver.HaltVelocityMovement(NODE_ID_LEFT, ulErrorCode);
    cEpos4driver.HaltVelocityMovement(NODE_ID_RIGHT, ulErrorCode);
    cEpos4driver.g_emergency = PAUSE_STOP;
    ss << "epos4_bardware_2wheel_node::pause stop" << endl;
    str = ss.str();
//    cLog.sendLogMsg(cLog.Warn, str);
    break;
  default:
    break;
  }
}

/*************************************************************************************************
 * sendActualVelocity
 * data[0] : left motor velocity
 * data[1] : right motor velocity
 *************************************************************************************************/
void sendActualVelocity()
{
  int ActualVel[NODE_CONUT]={};
  std_msgs::Int16MultiArray vel;
  std_msgs::Bool  error_msg;
  unsigned int ulErrorCode[4] = {0, 0};
  vel.data.resize(2);

  cEpos4driver.GetVelocityIsAveraged(NODE_ID_LEFT,  ActualVel[NODE_1], ulErrorCode[NODE_1]);
  cEpos4driver.GetVelocityIsAveraged(NODE_ID_RIGHT, ActualVel[NODE_2], ulErrorCode[NODE_2]);

  vel.data[NODE_1] = (int)ActualVel[NODE_1] * -1;
  vel.data[NODE_2] = (int)ActualVel[NODE_2];

  if((ulErrorCode[NODE_1] + ulErrorCode[NODE_2]) == 0) { error_msg.data = ERROR_OFF; }
  else { error_msg.data = ERROR_ON; }

  motor_vel_pub.publish(vel);
  error_pub.publish(error_msg);
}

/*************************************************************************************************
 * initParam
 * initialize epos driver parmater
 *************************************************************************************************/
int initParam()
{
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;

  for(int i=0; i<NODE_CONUT;i++)
  {
    motor_param[i].mode           = EPOS4_MODE_VELOCITY;
    motor_param[i].accelaration   = DRIVER_ACCELARATION;
    motor_param[i].deceleration   = DRIVER_DECELERATION;
  }

  cEpos4driver.SetDefaultParameters();
  cEpos4driver.OpenDevice(&ulErrorCode);
  cEpos4driver.EnableDevice(NODE_ID_LEFT,  &ulErrorCode);
  cEpos4driver.EnableDevice(NODE_ID_RIGHT, &ulErrorCode);
  cEpos4driver.SetProfileVelocityMode(NODE_ID_LEFT,  ulErrorCode);
  cEpos4driver.SetProfileVelocityMode(NODE_ID_RIGHT, ulErrorCode);
  cEpos4driver.SetVelocityProfile(NODE_ID_LEFT, motor_param[NODE_1].accelaration,
                                  motor_param[NODE_1].deceleration, ulErrorCode);
  cEpos4driver.SetVelocityProfile(NODE_ID_RIGHT, motor_param[NODE_2].accelaration,
                                  motor_param[NODE_2].deceleration, ulErrorCode);

  if(ulErrorCode == 0) lResult = MMC_SUCCESS;

  return lResult;
}

/*************************************************************************************************
 * closeDriver
 * close epos driver
 *************************************************************************************************/
bool closeDriver()
{
  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;

  cEpos4driver.MoveProfileVelocity(NODE_ID_LEFT,  0,   ulErrorCode);
  cEpos4driver.MoveProfileVelocity(NODE_ID_RIGHT, 0,  ulErrorCode);
  cEpos4driver.CloseDevice(&ulErrorCode);

  if(ulErrorCode == 0) lResult = MMC_SUCCESS;

  return lResult;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_control_node");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  ros::Rate r(100);

  motor_vel_pub     = nh.advertise<std_msgs::Int16MultiArray>("aiedu/mobile/encoder", 10);
  error_pub         = nh.advertise<std_msgs::Bool>("aiedu/mobile/error", 10);
  joy_cmd_sub       = nh.subscribe("aiedu/cmd_vel", 1, callbackJoyCmdMsg);
  motor_param_sub   = nh.subscribe("aiedu/mobile/param", 10, callbackMotorParamMsg);
  emergency_sub     = nh.subscribe("aiedu/mobile/emergency", 10, callbackEmergencyMsg);

//  cLog.setPublisher(nh);

  while(initParam() != MMC_SUCCESS) { sleep(5); }


  while(ros::ok())
  {
    sendActualVelocity();
    ros::spinOnce();
    r.sleep();
  }

  closeDriver();

  ros::shutdown();

  return true;
}


