/*******************************************
**
*******************************************/
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
//#include "clro_main/clro_base.h"


/*******************************************************
** define
********************************************************/
#define FREQUENCY         100
#define PI                3.141592
#define GEAR_RATE         8   // 51:1
#define WHEEL_NUM         2
#define WHEEL_RADIUS      0.0675        // meter
#define WHEEL_SEPARATION  0.28          // meter
#define TICK2RAD          0.0001717993  //0.0003665193   // 0.000349066    // (RPM/30/60)/10 * (2*3.1415926535*0.102)
                                        // (RPM*0.033333333*0.016666667)*0.1 * (2*3.1415926535*0.102)
                                        // (rpm/60)/100 * (2*PI*r) : RPS/10ms * distance of rotation.
#define INCH2M(x)         x*0.0254

#define TIMER_1S          100
#define TIMER_500MS       50
#define TIMER_200MS       20
#define TIMER_100MS       10

#define USE_DEVICE        true
#define NOT_USE_DEVICE    false

                 
enum MOTOR
{
  LEFT=0,
  RIGHT,
  MOTOR_NUM
};

enum CMD_MODE
{
  NON_MODE=0,
  JOYSTIC_MODE,
  TABLET_MODE,
  NAVI_MODE,
};

/*******************************************
** ros pub/sub
*******************************************/
ros::Publisher odom_pub;
ros::Publisher joint_states_pub;
ros::Publisher navi_pub;
ros::Publisher tf_pub;
ros::Publisher cmd_pub;
ros::Subscriber imu_sub;
ros::Subscriber motor_vel_sub;
ros::Subscriber joy_vel_sub;
ros::Subscriber tablet_vel_sub;
ros::Subscriber navi_vel_sub;


/*******************************************
** global value
*******************************************/
unsigned int gTime = 0;
unsigned int pTime[10] = {0};
int timer_count = 0;

double last_diff_tick[MOTOR_NUM];
double last_tick[MOTOR_NUM];
double last_rad[MOTOR_NUM];
double last_velocity[MOTOR_NUM];

bool init_encoder = true;

double motor_vel[MOTOR_NUM];

// Odometry of robot
bool  g_ParamUseImu;

// Odometry of robot
nav_msgs::Odometry odom;

// Calculate the tf
geometry_msgs::TransformStamped odom_tf;

// Declaration for SLAM and navigation
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

// Joint state of robot
sensor_msgs::JointState joint_states;
sensor_msgs::Imu imu_msg;
//AieduMain::AieduBase base;

// Joy topic
geometry_msgs::Twist joy_cmd_msg;
geometry_msgs::Twist tablet_cmd_msg;
geometry_msgs::Twist navi_cmd_msg;
geometry_msgs::Twist cmd_msg;
bool          joy_cmd_flag;
bool          tablet_cmd_flag;
bool          navi_cmd_flag;
bool          button_flag;
int           cmd_mode;
unsigned int  joy_cmd_count;
unsigned int  tablet_cmd_count;
unsigned int  navi_cmd_count;
unsigned int  prev_joy_cmd_count;
unsigned int  prev_tablet_cmd_count;
unsigned int  prev_navi_cmd_count;

/*******************************************
** initlize joint states
*******************************************/
void initJointStates()
{
  static char *joint_states_name[] = {(char *)"wheel_left_joint",
                                      (char *)"wheel_right_joint",};
  joint_states.name.resize(2);

  joint_states.header.frame_id    = "base_link";
  joint_states.name[LEFT]   = joint_states_name[LEFT];
  joint_states.name[RIGHT]  = joint_states_name[RIGHT];

  cmd_mode              = NON_MODE;
  joy_cmd_count         = 0;
  tablet_cmd_count      = 0;
  navi_cmd_count        = 0;
  prev_joy_cmd_count    = 0;
  prev_tablet_cmd_count = 0;
  prev_navi_cmd_count   = 0;
}

/*******************************************
** Calculate the odometry
*******************************************/
bool calcOdometry(double diff_time)
{
  float orientation[4] ={ 0.0 };
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

//  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
//  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  wheel_l = (double)last_diff_tick[LEFT];
  wheel_r = (double)last_diff_tick[RIGHT];

  if (std::isnan(wheel_l))
    wheel_l = 0.0;

  if (std::isnan(wheel_r))
    wheel_r = 0.0;

//  delta_s     = WHEEL_RADIUS * (wheel_l + wheel_r) / 2.0;
  delta_s     = (wheel_l + wheel_r) / 2.0;

  if(g_ParamUseImu)
  {
    theta = -1.0 * atan2f(imu_msg.orientation.x * imu_msg.orientation.y + imu_msg.orientation.w * imu_msg.orientation.z,
            0.5f - imu_msg.orientation.y * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.z);
    delta_theta = theta - last_theta;
  }
  else
  {
//    theta = (wheel_l - wheel_r) / WHEEL_SEPARATION;
    theta = (wheel_l - wheel_r) / WHEEL_SEPARATION * 0.72;
    delta_theta = theta;
  }



  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity[LEFT]   = wheel_l / step_time;
  last_velocity[RIGHT]  = wheel_r / step_time;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_theta = theta;

  ROS_INFO("calcOdometry");
  return true;
}

/*******************************************
** update motor infomation function
*******************************************/
void updateMotorInfo(double left_tick, double right_tick)
{
//  int current_tick = 0;

//  if (init_encoder)
//  {
//    for (int index = 0; index < MOTOR_NUM; index++)
//    {
//      last_diff_tick[index] = 0;
//      last_tick[index]      = 0;
//      last_rad[index]       = 0;
//      last_velocity[index]  = 0.0;
//    }

//    last_tick[LEFT] = left_tick;
//    last_tick[RIGHT] = right_tick;

//    init_encoder = false;
//    return;
//  }

//  current_tick = left_tick;

//  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
//  last_tick[LEFT]      = current_tick;
//  last_rad[LEFT]       += TICK2RAD * last_diff_tick[LEFT];

//  current_tick = right_tick;

//  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
//  last_tick[RIGHT]      = current_tick;
//  last_rad[RIGHT]       += TICK2RAD * last_diff_tick[RIGHT];

  double current_tick = 0;

  if (init_encoder)
  {
    for (int index = 0; index < MOTOR_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0;
      last_velocity[index]  = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += last_diff_tick[RIGHT];

  ROS_INFO("updateMotorInfo");
}

/*******************************************
** update function for odometry
*******************************************/
void updateOdometry()
{
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = odom_pose[2];
  odom.pose.pose.orientation.w = 0;

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  ROS_INFO("updateOdometry");
}

/*******************************************
** CalcUpdateulate the TF
*******************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf_msg)
{
  odom_tf_msg.header = odom.header;
  odom_tf_msg.child_frame_id = "base_footprint";
  odom_tf_msg.transform.translation.x = odom.pose.pose.position.x;
  odom_tf_msg.transform.translation.y = odom.pose.pose.position.y;
  odom_tf_msg.transform.translation.z = odom.pose.pose.position.z;
  odom_tf_msg.transform.rotation      = odom.pose.pose.orientation;

  ROS_INFO("updateTF");
}

/*******************************************
** Update the joint states
*******************************************/
void updateJointStates()
{
  static float joint_states_pos[MOTOR_NUM] = {0.0};
  static float joint_states_vel[MOTOR_NUM] = {0.0};
  static float joint_states_eff[MOTOR_NUM] = {0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position.resize(2);
  joint_states.velocity.resize(2);

  joint_states.position[LEFT]   = joint_states_pos[LEFT];
  joint_states.position[RIGHT]  = joint_states_pos[RIGHT];
  joint_states.velocity[LEFT]   = joint_states_vel[LEFT];
  joint_states.velocity[RIGHT]  = joint_states_vel[RIGHT];

  ROS_INFO("updateJointStates");
}

/*******************************************
** Publish msgs (odometry, joint states, tf)
*******************************************/
void publishDriveInformation()
{
  unsigned long time_now = gTime;
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now;
  stamp_now = ros::Time::now();

  // calculate odometry
  calcOdometry(step_time);

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_pub.publish(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(joint_states);

  ROS_INFO("publishDriveInformation");
}

/*******************************************
** ros imu msg subscriber function
*******************************************/
void msgImuCallback(const sensor_msgs::Imu msg)
{
  imu_msg = msg;
}

/*******************************************
** ros motor speed msg subscriver function
*******************************************/
void msgMotorSpeedCallback(const std_msgs::Int16MultiArray msg)
{
  // rpm 값으로 받음
  // 10ms 주기로 받음
  // 모터 rpm 기준으로 받음 -> 바퀴 기준 rpm  1/51
  // rpm 분당 회전수 -> 초당 = rpm/60
  // 10ms = rpm/60/100
  // 바퀴 기준 10ms = rpm/60/100/51

  // 원둘레 = 2*pi*r
  //       = 2*WHEEL_RADIUS*3.141592
  // 이동 거리 = rpm/60/100/51 * 2*WHEEL_RADIUS*3.141592

  double motor_left  = (double)msg.data[LEFT] / 60 / 100 / GEAR_RATE;
  double motor_right = (double)msg.data[RIGHT] / 60 / 100 / GEAR_RATE;

  motor_left =  motor_left * 2 * WHEEL_RADIUS * PI;
  motor_right =  motor_right * 2 * WHEEL_RADIUS * PI;

  motor_vel[LEFT]  += motor_left;  //msg.data[LEFT];
  motor_vel[RIGHT] += motor_right; //msg.data[RIGHT];

  updateMotorInfo(motor_vel[LEFT], motor_vel[RIGHT]);
  publishDriveInformation();
}

/*******************************************
** make cmd vel
*******************************************/
void makeJoyCmd()
{
  if(joy_cmd_flag)
  {
    cmd_msg = joy_cmd_msg;
    cmd_pub.publish(cmd_msg);
  }
  else if(tablet_cmd_flag)
  {
    cmd_msg.linear.x  = tablet_cmd_msg.linear.x / 2;
    cmd_msg.linear.y  = tablet_cmd_msg.linear.y / 2;
    cmd_msg.angular.z = tablet_cmd_msg.angular.z / 2;
    cmd_pub.publish(cmd_msg);
  }
  else if(navi_cmd_flag)
  {
    cmd_msg = navi_cmd_msg;
    cmd_pub.publish(cmd_msg);
  }
  else
  {
//    cmd_msg.linear.x  = 0;
//    cmd_msg.linear.y  = 0;
//    cmd_msg.linear.z  = 0;
//    cmd_msg.angular.x = 0;
//    cmd_msg.angular.y = 0;
//    cmd_msg.angular.z = 0;
  }

}

/*******************************************
** callback joystic msg function
*******************************************/
void callbackJoyCmdMsg(const geometry_msgs::Twist& msg) //1
{
  joy_cmd_msg = msg;
  joy_cmd_flag = true;
  joy_cmd_count++;
  // makeJoyCmd();
}

/*******************************************
** callback tablet msg function
*******************************************/
void callbackTabletCmdMsg(const geometry_msgs::Twist& msg)  //2
{
  tablet_cmd_msg = msg;
  tablet_cmd_flag = true;
  tablet_cmd_count++;
  // makeJoyCmd();
}

/*******************************************
** callback navi msg function
*******************************************/
void callbackNaviCmdMsg(const geometry_msgs::Twist& msg)  // 3
{
  navi_cmd_msg = msg;
  navi_cmd_flag = true;
  navi_cmd_count++;
  // makeJoyCmd();
}

void CmdMux()
{
  if(joy_cmd_count - prev_joy_cmd_count > 1)
  {
    joy_cmd_flag        = true;
    joy_cmd_count       = 0;
    prev_joy_cmd_count  = 0;
  }
  else
  {
    joy_cmd_flag        = false;
  }
  if(tablet_cmd_count - prev_tablet_cmd_count > 1)
  {
    tablet_cmd_flag       = true;
    tablet_cmd_count      = 0;
    prev_tablet_cmd_count = 0;
  }
  else
  {
    tablet_cmd_flag       = false;
  }
  if(navi_cmd_count - prev_navi_cmd_count > 1)
  {
    navi_cmd_flag       = true;
    navi_cmd_count      = 0;
    prev_navi_cmd_count = 0;
  }
  else
  {
    navi_cmd_flag       = false;
  }

}


/*******************************************
** main function
*******************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "aiedu_main_node");

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  ros::Rate r(FREQUENCY);

  odom_pub          = nh.advertise<nav_msgs::Odometry>("odom", 10);
  joint_states_pub  = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  navi_pub          = nh.advertise<geometry_msgs::PoseStamped>("sub_geometry_msgs", 10);
  tf_pub            = nh.advertise<geometry_msgs::TransformStamped>("robot_tf", 10);
  cmd_pub           = nh.advertise<geometry_msgs::Twist>("aiedu/cmd_vel", 1);
  imu_sub           = nh.subscribe("imu", 1, msgImuCallback);
  motor_vel_sub     = nh.subscribe("speed", 1, msgMotorSpeedCallback);
  joy_vel_sub       = nh.subscribe("joy/cmd_vel", 1, callbackJoyCmdMsg);
  tablet_vel_sub    = nh.subscribe("tablet/cmd_vel", 1, callbackTabletCmdMsg);
  navi_vel_sub      = nh.subscribe("navi/cmd_vel", 1, callbackNaviCmdMsg);

  initJointStates();

  g_ParamUseImu = NOT_USE_DEVICE;


  while( ros::ok() )
  {
    if((gTime - pTime[0]) >= TIMER_100MS)
    {
//      updateMotorInfo(motor_vel[LEFT], motor_vel[RIGHT]);
//      publishDriveInformation();
      pTime[0] = gTime;
    }
    if((gTime - pTime[1]) >= TIMER_500MS)
    {

      CmdMux();
      pTime[1] = gTime;
    }

    makeJoyCmd();
    ros::spinOnce();
    r.sleep();
    gTime++;
  }

  ros::shutdown();

  return 0;
}

