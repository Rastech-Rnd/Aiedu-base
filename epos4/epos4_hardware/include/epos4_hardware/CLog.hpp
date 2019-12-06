#ifndef EPOS4_HARDWARE_CLOG_HPP
#define EPOS4_HARDWARE_CLOG_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <string.h>
#include <sstream>
#include <std_msgs/String.h>

using namespace std;

class CLog
{
public:
  CLog();
  ~CLog();
  /*********************
  ** Logging
  **********************/
  enum LogLevel { Debug, Info, Warn, Error, Fatal };
  void sendLogMsg( const LogLevel &level, const string &msg);
  void setPublisher(ros::NodeHandle nh, const char *log_topic_name = "/log");
private:

public:
private:
  ros::NodeHandle nh_;
  ros::Publisher  log_pub_;

};


#endif

