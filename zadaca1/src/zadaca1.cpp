#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

// CALLBACK FUNCTION
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // GET ROBOT POSSITION
  ROS_INFO("Position-> x: [%f], y: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zadaca1");

  // NodeHandle - MAIN ACCESS POINT TO COMMUNICATE WITH ROS SYSTEM
  ros::NodeHandle n;

  // RECEIVE MESSAGES FROM TOPIC "/odom", BUFFER SIZE - 1000
  ros::Subscriber sub = n.subscribe("/odom", 1000, chatterCallback);

  // ros::spin() EXIT WHEN Ctrl-C IS PRESSED
  ros::spin();

  return 0;
}
