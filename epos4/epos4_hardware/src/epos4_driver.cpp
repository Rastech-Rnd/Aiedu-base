#include <epos4_hardware/epos4_driver.hpp>

CEpos4driver::CEpos4driver()
{
}

CEpos4driver::~CEpos4driver()
{
  unsigned int ulErrorCode = 0;
  CloseDevice(&ulErrorCode);
}

void CEpos4driver::LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{ 
  stringstream ss;
  ss << g_programName << ": " << functionName << " failed (result="
     << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;

  ROS_ERROR_STREAM(ss.str());
}

void CEpos4driver::LogInfo(string message)
{
  ROS_INFO_STREAM(message);
}

void CEpos4driver::SetDefaultParameters()
{
  //USB
  g_usNodeId = 1;
  g_deviceName = "EPOS4";
  g_protocolStackName = "MAXON SERIAL V2";
  g_interfaceName = "USB";
  g_portName = "USB0";
  g_baudrate = 1000000;
}

int CEpos4driver::OpenDevice(unsigned int* p_pErrorCode)
{
  int lResult = MMC_FAILED;

  char* pDeviceName = new char[255];
  char* pProtocolStackName = new char[255];
  char* pInterfaceName = new char[255];
  char* pPortName = new char[255];

  strcpy(pDeviceName, g_deviceName.c_str());
  strcpy(pProtocolStackName, g_protocolStackName.c_str());
  strcpy(pInterfaceName, g_interfaceName.c_str());
  strcpy(pPortName, g_portName.c_str());

  LogInfo("Open device...");

  g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

  if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
  {
    unsigned int lBaudrate = 0;
    unsigned int lTimeout = 0;

    if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
    {
      if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
      {
        if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
          if(g_baudrate==(int)lBaudrate)
          {
            lResult = MMC_SUCCESS;
          }
        }
      }
    }
  }
  else
  {
    g_pKeyHandle = 0;
  }

  delete []pDeviceName;
  delete []pProtocolStackName;
  delete []pInterfaceName;
  delete []pPortName;

  return lResult;
}

int CEpos4driver::CloseDevice(unsigned int* p_pErrorCode)
{
  int lResult = MMC_FAILED;

  *p_pErrorCode = 0;

  LogInfo("Close device");

  if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
  {
    lResult = MMC_SUCCESS;
  }

  return lResult;
}

int CEpos4driver::EnableDevice(unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
  int lResult = MMC_SUCCESS;
  BOOL oIsFault = 0;

  if(VCS_GetFaultState(g_pKeyHandle, p_usNodeId, &oIsFault, p_pErrorCode ) == 0)
  {
    LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
    lResult = MMC_FAILED;
  }

  if(lResult==0)
  {
    if(oIsFault)
    {
      stringstream msg;
      msg << "clear fault, node = '" << p_usNodeId << "'";
      LogInfo(msg.str());

      if(VCS_ClearFault(g_pKeyHandle, p_usNodeId, p_pErrorCode) == 0)
      {
        LogError("VCS_ClearFault", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
      }
    }

    if(lResult==0)
    {
      BOOL oIsEnabled = 0;

      if(VCS_GetEnableState(g_pKeyHandle, p_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
      {
        LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
      }

      if(lResult==0)
      {
        if(!oIsEnabled)
        {
          if(VCS_SetEnableState(g_pKeyHandle, p_usNodeId, p_pErrorCode) == 0)
          {
            LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
          }
        }
      }
    }
  }
  return lResult;
}

int CEpos4driver::DisableDevice(unsigned short p_usNodeId, unsigned int &p_pErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_SetDisableState(g_pKeyHandle, p_usNodeId, &p_pErrorCode) == 0)
  {
    LogError("VCS_SetDisableState", lResult, p_pErrorCode);
    lResult = MMC_FAILED;
  }
  else
  {
  }

  return lResult;
}

int CEpos4driver::SetProfileVelocityMode(unsigned short p_usNodeId, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_ActivateProfileVelocityMode(g_pKeyHandle, p_usNodeId, &p_ulErrorCode) == 0)
  {
    LogError("VCS_ActivateProfileVelocityMode", lResult, p_ulErrorCode);
    lResult = MMC_FAILED;
  }
  else
  {
    msg << "set profile velocity mode, node = " << p_usNodeId;
    LogInfo(msg.str());
  }

  return lResult;
}

int CEpos4driver::SetProfilePositionMode(unsigned short p_usNodeId,
                                         unsigned int & p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_ActivateProfilePositionMode(g_pKeyHandle, p_usNodeId, &p_ulErrorCode) == 0)
  {
    LogError("VCS_ActivateProfilePositionMode", lResult, p_ulErrorCode);
    lResult = MMC_FAILED;
  }
  else
  {
    msg << "set profile position mode, node = " << p_usNodeId;
    LogInfo(msg.str());
  }

  return lResult;
}

int CEpos4driver::SetProfileHomingMode(unsigned short p_usNodeId, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_ActivateHomingMode(g_pKeyHandle, p_usNodeId, &p_ulErrorCode) == 0)
  {
    LogError("VCS_ActivateHomingMode", lResult, p_ulErrorCode);
    lResult = MMC_FAILED;
  }
  else
  {
    msg << "set profile homing mode, node = " << p_usNodeId;
    LogInfo(msg.str());
  }

  return lResult;
}

int CEpos4driver::SetVelocityProfile(unsigned short p_usNodeId, unsigned int p_ulProfileAcceleration,
                                     unsigned int p_ulProfileDeceleration, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_SetVelocityProfile(g_pKeyHandle, p_usNodeId, p_ulProfileAcceleration, p_ulProfileDeceleration, &p_ulErrorCode) == 0)
  {
    LogError("VCS_SetVelocityProfile", lResult, p_ulErrorCode);
    lResult = MMC_FAILED;
  }
  else
  {
    msg << "set velocity profile, node = " << p_usNodeId  << ", acceleration = "
        << p_ulProfileAcceleration << ", deceleration = " << p_ulProfileDeceleration;
    LogInfo(msg.str());
  }

  return lResult;
}

int CEpos4driver::SetPositionProfile(unsigned short p_usNodeId, unsigned int p_ulProfileVelocity,
                                     unsigned int p_ulProfileAcceleration, unsigned int p_ulProfileDeceleration,
                                     unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_SetPositionProfile(g_pKeyHandle, p_usNodeId, p_ulProfileVelocity,
                            p_ulProfileAcceleration, p_ulProfileDeceleration, &p_ulErrorCode) == 0)
  {
    LogError("VCS_SetPositionProfile", lResult, p_ulErrorCode);
    lResult = MMC_FAILED;
  }
  else
  {
    msg << "set position profile, node = " << p_usNodeId << ", velocity = " << p_ulProfileVelocity
        << ", acceleration = " << p_ulProfileAcceleration << ", deceleration = " << p_ulProfileDeceleration;
    LogInfo(msg.str());
  }

  return lResult;
}

/**************************************************************************************************
 * MoveProfilePostion
 * p_usNodeId = node id
 * p_lTargetPosition = target position
 *************************************************************************************************/
int CEpos4driver::MoveProfileVelocity(unsigned short p_usNodeId, long p_lTargetVelocity, unsigned int & p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(g_emergency == EMERGENCY_OFF)
  {
    if(VCS_MoveWithVelocity(g_pKeyHandle, p_usNodeId, p_lTargetVelocity, &p_ulErrorCode) == 0)
    {
      lResult = MMC_FAILED;
      LogError("VCS_MoveWithVelocity", lResult, p_ulErrorCode);
    }
    else
    {
      msg << "move with target velocity = " << p_lTargetVelocity << " rpm, node = " << p_usNodeId;
      LogInfo(msg.str());
    }
  }
  /***********************************************************************************************
  * 데이터 버퍼에 저장 및 앞에서부터 가져다 쓰기
 //    list<long> velocityList;

 //    velocityList.push_back(100);
 //    velocityList.push_back(500);
 //    velocityList.push_back(1000);

 //    for(list<long>::iterator it = velocityList.begin(); it !=velocityList.end(); it++)
 //    {
 //      long targetvelocity = (*it);

 //      stringstream msg;
 //      msg << "move with target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
 //      LogInfo(msg.str());

 //      if(VCS_MoveWithVelocity(g_pKeyHandle, p_usNodeId, targetvelocity, &p_ulErrorCode) == 0)
 //      {
 //        lResult = MMC_FAILED;
 //        LogError("VCS_MoveWithVelocity", lResult, p_ulErrorCode);
 //        break;
 //      }

 //      sleep(1);
 //    }
  **********************************************************************************************/

  return lResult;
}

/**************************************************************************************************
 * MoveProfilePostion
 * p_usNodeId = node id
 * p_lTargetPosition = target position
 * p_lAbsolute = FALSE:relative , TRUE:absolute
 * p_lImmediately = TRUE: starts immediately, FALSE: waits to end of last positioning
 *************************************************************************************************/
int CEpos4driver::MoveProfilePostion(unsigned short p_usNodeId, long p_lTargetPosition,
                                     int p_lAbsolute, int p_lImmediately, unsigned int & p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(g_emergency == EMERGENCY_OFF)
  {
    if(VCS_MoveToPosition(g_pKeyHandle, p_usNodeId, p_lTargetPosition,
                          p_lAbsolute, p_lImmediately, &p_ulErrorCode) == 0)
    {
      lResult = MMC_FAILED;
      LogError("VCS_MoveWithVelocity", lResult, p_ulErrorCode);
    }
    else
    {
      msg << "move to position = " << p_lTargetPosition << ", node = " << p_usNodeId;
      LogInfo(msg.str());
    }
  }

  return lResult;
}

int CEpos4driver::HaltVelocityMovement(unsigned short p_usNodeId, unsigned int & p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_HaltVelocityMovement(g_pKeyHandle, p_usNodeId, &p_ulErrorCode) == 0)
  {
    lResult = MMC_FAILED;
    LogError("VCS_HaltVelocityMovement", lResult, p_ulErrorCode);
  }
  else
  {
    msg << "halt velocity movement node = " << p_usNodeId;
    LogInfo(msg.str());
  }

  return lResult;
}

int CEpos4driver::HaltPositionMovement(unsigned short p_usNodeId, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_HaltPositionMovement(g_pKeyHandle, p_usNodeId, &p_ulErrorCode) == 0)
  {
    lResult = MMC_FAILED;
    LogError("VCS_HaltVelocityMovement", lResult, p_ulErrorCode);
  }
  else
  {
    msg << "halt position movement node = " << p_usNodeId;
    LogInfo(msg.str());
  }

  return lResult;
}

int CEpos4driver::GetVelocityIsAveraged(unsigned short p_usNodeId, int &p_lVelocityIsAveraged, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  int vel;
  stringstream msg;

  if(VCS_GetVelocityIsAveraged(g_pKeyHandle, p_usNodeId, &p_lVelocityIsAveraged, &p_ulErrorCode) == 0)
  {
    lResult = MMC_FAILED;
    LogError("VCS_GetVelocityIsAveraged", lResult, p_ulErrorCode);
  }
  else
  {
    msg << "get actual velocity = " << p_lVelocityIsAveraged << ", node = " << p_usNodeId;
    LogInfo(msg.str());
    lResult = p_lVelocityIsAveraged;

  }

  return lResult;

}

int CEpos4driver::GetActualPosition(unsigned short p_usNodeId, int &p_lPositionIs, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_GetPositionIs(g_pKeyHandle, p_usNodeId, &p_lPositionIs, &p_ulErrorCode) == 0)
  {
    lResult = MMC_FAILED;
    LogError("VCS_GetPositionIs", lResult, p_ulErrorCode);
  }
  else
  {
    msg << "get actual position = " << p_lPositionIs << ", node = " << p_usNodeId;
    LogInfo(msg.str());
    lResult = p_lPositionIs;
  }

  return lResult;
}

int CEpos4driver::GetMovementState(unsigned short p_usNodeId, int &p_pTargetReached, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_GetMovementState(g_pKeyHandle, p_usNodeId, &p_pTargetReached, &p_ulErrorCode) == 0)
  {
    lResult = MMC_FAILED;
    LogError("VCS_GetMovementState", lResult, p_ulErrorCode);
  }
  else
  {
    msg << "get movement state = " << p_pTargetReached << ", node = " << p_usNodeId;
    LogInfo(msg.str());
    lResult = p_pTargetReached;
  }

  return lResult;
}

int CEpos4driver::DefinePosition(unsigned short p_usNodeId,
                                 int p_lHomePosition, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_DefinePosition(g_pKeyHandle, p_usNodeId, p_lHomePosition, &p_ulErrorCode) == 0)
  {
    lResult = MMC_FAILED;
    LogError("VCS_DefinePosition", lResult, p_ulErrorCode);
  }
  else
  {
    msg << "define home position = " << p_lHomePosition << ", node = " << p_usNodeId;
    LogInfo(msg.str());
  }

  return lResult;
}

int CEpos4driver::EmergencyStop(unsigned short p_usNodeId, unsigned int &p_ulErrorCode)
{
  int lResult = MMC_SUCCESS;
  stringstream msg;

  if(VCS_SetQuickStopState(g_pKeyHandle, p_usNodeId, &p_ulErrorCode) == 0)
  {
    lResult = MMC_FAILED;
    LogError("VCS_SetQuickStopState", lResult, p_ulErrorCode);
  }
  else
  {
    msg << "Quick Stop node = " << p_usNodeId;
    LogInfo(msg.str());
  }

  return lResult;
}
