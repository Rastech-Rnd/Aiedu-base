#include "epos4_hardware/CLog.hpp"

CLog::CLog() 
{
  // log_pub_  = nh_.advertise<std_msgs::String>(str_name, 10);
}

CLog::~CLog()
{
}

void CLog::sendLogMsg(const LogLevel &level, const string &msg)
{
  std::stringstream logging_model_msg;
  std_msgs::String  log_msgs;

  switch ( level )
  {
  case(Debug) :
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  case(Info) :
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  case(Warn) :
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
    break;
  case(Error) :
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  case(Fatal) :
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  default:
    return;
  }

  log_msgs.data = logging_model_msg.str();
  log_pub_.publish(log_msgs);
}

void CLog::setPublisher(ros::NodeHandle nh, const char *log_topic_name) 
{
  string str_name(log_topic_name);
  log_pub_  = nh_.advertise<std_msgs::String>(str_name, 10);
}


