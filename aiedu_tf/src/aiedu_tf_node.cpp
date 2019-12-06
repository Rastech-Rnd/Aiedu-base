#include <ros/ros.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>



void poseCallback(const geometry_msgs::TransformStamped& msg)
{
  geometry_msgs::TransformStamped pubMsg;

  static tf2_ros::TransformBroadcaster br;
  tf2::Quaternion q;

  pubMsg.header.stamp = msg.header.stamp;
  pubMsg.header.frame_id = msg.header.frame_id;
  pubMsg.child_frame_id = msg.child_frame_id;
  pubMsg.transform.translation.x = msg.transform.translation.x;
  pubMsg.transform.translation.y = msg.transform.translation.y;
  pubMsg.transform.translation.z = msg.transform.translation.z;
  q.setRPY(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z);
  pubMsg.transform.rotation.x = q.x();
  pubMsg.transform.rotation.y = q.y();
  pubMsg.transform.rotation.z = q.z();
  pubMsg.transform.rotation.w = q.w();

  br.sendTransform(pubMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aiedu_tf");
  ros::NodeHandle nh;

  ros::Rate r(100);

  ros::Subscriber sub = nh.subscribe("/robot_tf", 10, &poseCallback);

  while(nh.ok())
  {
    ros::spin();
  }

  return 0;
}



