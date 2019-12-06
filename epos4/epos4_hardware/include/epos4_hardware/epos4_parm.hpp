#ifndef CEPOS4PARAM_HPP
#define CEPOS4PARAM_HPP

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
#define MMC_MAX_LOG_MSG_SIZE 512
#endif

#define EPOS4_MODE_VELOCITY                 0
#define EPOS4_MODE_POSITION                 1
#define EPOS4_PARAM_VELOCITY                1000
#define EPOS4_PARAM_ACCELARATION            2000
#define EPOS4_PARAM_DECELERATION            2000
#define HOME_POSITION                       0

#define MAXON_EC_MOTOR_100_POS_MAX          2400
#define MAXON_EC_MOTOR_100_POS_MIN          0
#define MAXON_EC_MOTOR_71_POS_MAX           1680
#define MAXON_EC_MOTOR_71_POS_MIN           0
#define MAXON_EC_MOTOR_FUALHABER_POS_MAX    1024
#define MAXON_EC_MOTOR_FUALHABER_POS_MIN    0
#define ARM_CONTOL_RANGE_MAX                900
#define ARM_CONTOL_RANGE_MIN               -900


struct epos4_param
{
  int mode;
  int velocity;
  int accelaration;
  int deceleration;
};

enum LeftRight
{
  LEFT = 0,
  RIGHT,
};

enum Epos4NodeID
{
  NODE_1,
  NODE_2,
  NODE_3,
  NODE_4,
  NODE_NUM,
};

enum EAppMode
{
  AM_UNKNOWN,
  AM_DEMO,
  AM_INTERFACE_LIST,
  AM_PROTOCOL_LIST,
  AM_VERSION_INFO
};

enum Absolute_Mode
{
  RELATIVE = 0,
  ABSOLUTE,
};

enum Immediately_mode
{
  WAITS_POSITION = 0,
  IMMEDIATELY,
};

enum epos4_mode
{
  VELOCITY_MODE = 0,
  POSITION_MODE,
  HOMING_MODE,
};

enum epos4_emergency
{
  EMERGENCY_OFF = 0,
  EMERGENCY_STOP,
  PAUSE_STOP,
};

enum epos4_param_num
{
  VELOCITY_NUM,
  ACCELARATION_NUM,
  DECELERATION_NUM,
};



#endif
