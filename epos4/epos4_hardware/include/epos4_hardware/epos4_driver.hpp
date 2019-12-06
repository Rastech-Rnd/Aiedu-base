#ifndef CEPOS4DRIVER_HPP
#define CEPOS4DRIVER_HPP

#include <ros/ros.h>
#include "epos4_library/Definitions.h"
#include "epos4_hardware/epos4_parm.hpp"
#include <string.h>
#include <sstream>
#include <iostream>
#include <sstream>


using namespace std;

typedef void* HANDLE;
typedef int BOOL;


class CEpos4driver
{
public:
  CEpos4driver();
  ~CEpos4driver();

public:
  void* g_pKeyHandle = 0;
  unsigned short g_usNodeId = 1;
  string g_deviceName;
  string g_protocolStackName;
  string g_interfaceName;
  string g_portName;
  int g_baudrate = 0;
  EAppMode g_eAppMode = AM_DEMO;
  const string g_programName = "EPOS4Cmd";
  int g_emergency = 0;

public:
  void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
  void  LogInfo(string message);
  void  SetDefaultParameters();
  int   OpenDevice(unsigned int* p_pErrorCode);
  int   CloseDevice(unsigned int* p_pErrorCode);
  int   EnableDevice(unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int   DisableDevice(unsigned short p_usNodeId, unsigned int & p_pErrorCode);
  int   SetProfileVelocityMode(unsigned short p_usNodeId, unsigned int & p_ulErrorCode);
  int   SetProfilePositionMode(unsigned short p_usNodeId, unsigned int & p_ulErrorCode);
  int   SetProfileHomingMode(unsigned short p_usNodeId, unsigned int & p_ulErrorCode);
  int   SetVelocityProfile(unsigned short p_usNodeId, unsigned int p_ulProfileAcceleration,
                           unsigned int p_ulProfileDeceleration, unsigned int & p_ulErrorCode);
  int   SetPositionProfile(unsigned short p_usNodeId, unsigned int p_ulProfileVelocity,
                           unsigned int p_ulProfileAcceleration, unsigned int p_ulProfileDeceleration, unsigned int & p_ulErrorCode);
  int   MoveProfileVelocity(unsigned short p_usNodeId, long p_lTargetVelocity, unsigned int & p_ulErrorCode);
  int   MoveProfilePostion(unsigned short p_usNodeId, long p_lTargetPosition,
                           int p_lAbsolute, int p_lImmediately, unsigned int & p_ulErrorCode);
  int   HaltVelocityMovement(unsigned short p_usNodeId, unsigned int & p_ulErrorCode);
  int   HaltPositionMovement(unsigned short p_usNodeId, unsigned int & p_ulErrorCode);
  int   GetVelocityIsAveraged(unsigned short p_usNodeId, int & p_lVelocityIsAveraged, unsigned int & p_ulErrorCode);
  int   GetActualPosition(unsigned short p_usNodeId, int & p_lPositionIs, unsigned int & p_ulErrorCode);
  int   GetMovementState(unsigned short p_usNodeId, int & p_pTargetReached, unsigned int & p_ulErrorCode);
  int   DefinePosition(unsigned short p_usNodeId, int p_lHomePosition, unsigned int & p_ulErrorCode);
  int   EmergencyStop(unsigned short p_usNodeId, unsigned int & p_ulErrorCode);

// inputs position  quick stop deceleration

//status position
// target reached, setpoint ack, following error, position referenced to home position

//input velocity
// target velocity, profile acceleration, profile deceleration, quick stop deceleration

//status velocity
//target reached, speed is equal 0
};

#endif
