//============================================================================
// Name        : epos4_hardware_node.cpp
// Author      : Jhung JoonYoung
// Version     : 1.0
// Copyright   : maxon motor ag 2014-2018
// Description : epos4 driver control in ROS C++
//============================================================================
#include "epos4_hardware/epos4_driver.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <robot_msgs/NanaJoint.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <vector>


using namespace std;


/*************************************************************************************************
 * Define Motor position
 *************************************************************************************************/
#define NODE_ID_FRONT_LEFT    1
#define NODE_ID_FRONT_RIGHT   2
#define NODE_ID_REAR_LEFT     3
#define NODE_ID_REAR_RIGHT    4
#define NODE_CONUT            4

/*************************************************************************************************
 * Define Motor param
 *************************************************************************************************/
#define FOUR_WHEEL_DRIVER_ACCELARATION    6000
#define FOUR_WHEEL_DRIVER_DECELERATION    6000

#define WHEEL_RADIUS                      0.204 // m
#define WHEEL_SEPARATION_WIDTH            0.5   // m
#define WHEEL_SEPARATION_LENGTH           0.4   // m

#define ERROR_ON  true;
#define ERROR_OFF false;

CEpos4driver cEpos4driver;

ros::Publisher  motor_vel_pub;
ros::Publisher  error_pub;
ros::Subscriber joy_cmd_sub;
ros::Subscriber motor_param_sub;
ros::Subscriber emergency_sub;

epos4_param motor_param[NODE_CONUT];

int driver_mode = POSITION_MODE;
bool flagThread = false;

double wheel_front_left;
double wheel_front_right;
double wheel_rear_left;
double wheel_rear_right;



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
 * callbackJoyCmdMsg
 * get joystic commend and send motor speed
 *************************************************************************************************/
void callbackJoyCmdMsg(const geometry_msgs::Twist& msg)
{
  unsigned int ulErrorCode = 0;

  wheel_front_left  = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z);
  wheel_front_right = (1/WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z);
  wheel_rear_left   = (1/WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z);
  wheel_rear_right  = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z);

  cEpos4driver.MoveProfileVelocity(NODE_ID_FRONT_LEFT,  wheel_front_left * -600,   ulErrorCode);
  cEpos4driver.MoveProfileVelocity(NODE_ID_FRONT_RIGHT, wheel_front_right * 600,  ulErrorCode);
  cEpos4driver.MoveProfileVelocity(NODE_ID_REAR_LEFT,   wheel_rear_left * -600,    ulErrorCode);
  cEpos4driver.MoveProfileVelocity(NODE_ID_REAR_RIGHT,  wheel_rear_right * 600,   ulErrorCode);

  ROS_INFO("---------------------------------");
  ROS_INFO("wheel front left vel  : %f", wheel_front_left);
  ROS_INFO("wheel front right vel : %f", wheel_front_right);
  ROS_INFO("wheel rear left vel   : %f", wheel_rear_left);
  ROS_INFO("wheel rear right vel  : %f", wheel_rear_right);
  ROS_INFO("---------------------------------");
}

/*************************************************************************************************
 * callbackMotorParamMsg
 * data[0] : data = 0 -> all , 1~4 -> node number
 * data[1] : parameter accelaration
 * data[2] : parameter deceleration
 *************************************************************************************************/
void callbackMotorParamMsg(const std_msgs::Int32MultiArray& msg)
{
  unsigned int ulErrorCode = 0;

  if(msg.data[0] == 0)
  {
    for(int i=0; i<NODE_CONUT; i++)
    {
      cEpos4driver.SetVelocityProfile(i+1, msg.data[1], msg.data[2], ulErrorCode);
    }
  }
  else
  {
    cEpos4driver.SetVelocityProfile(msg.data[0], msg.data[1], msg.data[2], ulErrorCode);
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
  unsigned int ulErrorCode = 0;
  switch (data.data)
  {
  case EMERGENCY_OFF:
    cEpos4driver.g_emergency = EMERGENCY_OFF;
    break;
  case EMERGENCY_STOP:
    cEpos4driver.EmergencyStop(NODE_ID_FRONT_LEFT, ulErrorCode);
    cEpos4driver.EmergencyStop(NODE_ID_FRONT_RIGHT, ulErrorCode);
    cEpos4driver.EmergencyStop(NODE_ID_REAR_LEFT, ulErrorCode);
    cEpos4driver.EmergencyStop(NODE_ID_REAR_RIGHT, ulErrorCode);
    cEpos4driver.g_emergency = EMERGENCY_STOP;
    break;
  case PAUSE_STOP:
    cEpos4driver.HaltVelocityMovement(NODE_ID_FRONT_LEFT, ulErrorCode);
    cEpos4driver.HaltVelocityMovement(NODE_ID_FRONT_RIGHT, ulErrorCode);
    cEpos4driver.HaltVelocityMovement(NODE_ID_REAR_LEFT, ulErrorCode);
    cEpos4driver.HaltVelocityMovement(NODE_ID_REAR_RIGHT, ulErrorCode);
    cEpos4driver.g_emergency = PAUSE_STOP;
    break;
  default:
    break;
  }
}

/*************************************************************************************************
 * sendActualVelocity
 * data[0] : Front left motor velocity
 * data[1] : Front right motor velocity
 * data[3] : Back left motor velocity
 * data[4] : Back right motor velocity
 *************************************************************************************************/
void sendActualVelocity()
{
  int ActualVel[NODE_CONUT]={};
  std_msgs::Int16MultiArray vel;
  std_msgs::Bool  error_msg;
  unsigned int ulErrorCode[4] = {0, 0, 0, 0};
  vel.data.resize(4);

  cEpos4driver.GetVelocityIsAveraged(NODE_ID_FRONT_LEFT,  ActualVel[NODE_1], ulErrorCode[0]);
  cEpos4driver.GetVelocityIsAveraged(NODE_ID_FRONT_RIGHT, ActualVel[NODE_2], ulErrorCode[1]);
  cEpos4driver.GetVelocityIsAveraged(NODE_ID_REAR_LEFT,   ActualVel[NODE_3], ulErrorCode[2]);
  cEpos4driver.GetVelocityIsAveraged(NODE_ID_REAR_RIGHT,  ActualVel[NODE_4], ulErrorCode[3]);

  vel.data[NODE_1] = (int)ActualVel[NODE_1] * -1;
  vel.data[NODE_2] = (int)ActualVel[NODE_2];
  vel.data[NODE_3] = (int)ActualVel[NODE_3] * -1;
  vel.data[NODE_4] = (int)ActualVel[NODE_4];

  if((ulErrorCode[0] + ulErrorCode[0] + ulErrorCode[0] + ulErrorCode[0]) == 0) { error_msg.data = ERROR_OFF; }
  else { error_msg.data = ERROR_ON; }

  motor_vel_pub.publish(vel);
  error_pub.publish(error_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mecanum_control_node");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  ros::Duration d(0.01); // 0.01 = 100Hz   0.1 = 10Hz   1 = 1Hz

  for(int i=0; i<NODE_CONUT;i++)
  {
    motor_param[i].mode           = EPOS4_MODE_VELOCITY;
    motor_param[i].accelaration   = FOUR_WHEEL_DRIVER_ACCELARATION;
    motor_param[i].deceleration   = FOUR_WHEEL_DRIVER_DECELERATION;
  }

  motor_vel_pub     = nh.advertise<std_msgs::Int16MultiArray>("/fero/mobile/encoder", 10);
  error_pub         = nh.advertise<std_msgs::Bool>("fero/mobile/error", 10);
  joy_cmd_sub       = nh.subscribe("/fero/cmd_vel", 1, callbackJoyCmdMsg);
  motor_param_sub   = nh.subscribe("/fero/mobile/param", 10, callbackMotorParamMsg);
  emergency_sub     = nh.subscribe("/fero/mobile/emergency", 10, callbackEmergencyMsg);

  int lResult = MMC_FAILED;
  unsigned int ulErrorCode = 0;

  cEpos4driver.SetDefaultParameters();

  cEpos4driver.OpenDevice(&ulErrorCode);

  cEpos4driver.EnableDevice(NODE_ID_FRONT_LEFT,  &ulErrorCode);
  cEpos4driver.EnableDevice(NODE_ID_FRONT_RIGHT, &ulErrorCode);
  cEpos4driver.EnableDevice(NODE_ID_REAR_LEFT,   &ulErrorCode);
  cEpos4driver.EnableDevice(NODE_ID_REAR_RIGHT,  &ulErrorCode);
  cEpos4driver.SetProfileVelocityMode(NODE_ID_FRONT_LEFT,  ulErrorCode);
  cEpos4driver.SetProfileVelocityMode(NODE_ID_FRONT_RIGHT, ulErrorCode);
  cEpos4driver.SetProfileVelocityMode(NODE_ID_REAR_LEFT,   ulErrorCode);
  cEpos4driver.SetProfileVelocityMode(NODE_ID_REAR_RIGHT,  ulErrorCode);

  cEpos4driver.SetVelocityProfile(NODE_ID_FRONT_LEFT, motor_param[NODE_1].accelaration,
                                  motor_param[NODE_1].deceleration, ulErrorCode);
  cEpos4driver.SetVelocityProfile(NODE_ID_FRONT_RIGHT, motor_param[NODE_2].accelaration,
                                  motor_param[NODE_2].deceleration, ulErrorCode);
  cEpos4driver.SetVelocityProfile(NODE_ID_REAR_LEFT, motor_param[NODE_3].accelaration,
                                  motor_param[NODE_3].deceleration, ulErrorCode);
  cEpos4driver.SetVelocityProfile(NODE_ID_REAR_RIGHT, motor_param[NODE_4].accelaration,
                                  motor_param[NODE_4].deceleration, ulErrorCode);

  while(ros::ok())
  {
    sendActualVelocity();
    ros::spinOnce();
    d.sleep();
  }

  cEpos4driver.MoveProfileVelocity(NODE_ID_FRONT_LEFT,  0,   ulErrorCode);
  cEpos4driver.MoveProfileVelocity(NODE_ID_FRONT_RIGHT, 0,  ulErrorCode);
  cEpos4driver.MoveProfileVelocity(NODE_ID_REAR_LEFT,   0,    ulErrorCode);
  cEpos4driver.MoveProfileVelocity(NODE_ID_REAR_RIGHT,  0,   ulErrorCode);

  cEpos4driver.CloseDevice(&ulErrorCode);

  ros::shutdown();

  return lResult;
}

