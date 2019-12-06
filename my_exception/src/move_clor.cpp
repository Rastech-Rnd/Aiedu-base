#include <ros/ros.h>
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "std_srvs/Empty.h"
#include <cstdlib>


ros::Publisher goalpoint_pub;
ros::ServiceClient clro_map_clean;
std_srvs::Empty cleansrv;
move_base_msgs::MoveBaseActionGoal goalpos;
int error_count = 0;


void move_point(const move_base_msgs::MoveBaseActionResult msg)
{
    switch(msg.status.status)
    {
	case 3 :
	error_count = 0;
	break;
	case 4 : 
	error_count++;
	//if(error_count == 3){  }
        clro_map_clean.call(cleansrv);
        goalpoint_pub.publish(goalpos);
	break;
        case 11 :
        goalpos.goal.target_pose.header.frame_id = "map";
        goalpos.goal.target_pose.pose.position.x = -0.2843;
        goalpos.goal.target_pose.pose.position.y = -0.4884;
        goalpos.goal.target_pose.pose.position.z = 0.0;
        goalpos.goal.target_pose.pose.orientation.z = 0.0687;
        goalpos.goal.target_pose.pose.orientation.w = 0.9976;
        goalpoint_pub.publish(goalpos);
        break;
        case 12 :
        goalpos.goal.target_pose.header.frame_id = "map";
        goalpos.goal.target_pose.pose.position.x = 1.5875;
        goalpos.goal.target_pose.pose.position.y = -0.2615;
        goalpos.goal.target_pose.pose.position.z = 0.0;
        goalpos.goal.target_pose.pose.orientation.z = 0.9964;
        goalpos.goal.target_pose.pose.orientation.w = -0.0839;
        goalpoint_pub.publish(goalpos);
        break;
        /*case 13 :
        goalpos.goal.target_pose.header.frame_id = "map";
        goalpos.goal.target_pose.pose.position.x = 0.63;
        goalpos.goal.target_pose.pose.position.y = -2.81;
        goalpos.goal.target_pose.pose.position.z = 0.0;
        goalpos.goal.target_pose.pose.orientation.z = -0.483;
        goalpos.goal.target_pose.pose.orientation.w = 0.875;
        goalpoint_pub.publish(goalpos);
        break;*/
    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_clro");
  
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("/move_base/result", 1, move_point);

  clro_map_clean = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  goalpoint_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",1);

  while( ros::ok() )
  {
    ros::spinOnce();
  }

  return 0;
}
