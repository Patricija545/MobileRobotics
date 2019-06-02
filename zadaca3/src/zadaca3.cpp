#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <std_srvs/Empty.h>

double distanceToGoal;

bool flag = false;
bool flag_service = false;

struct coordinates {
	double x;
	double y;
	double z;
	double angle;
};

struct robot_speed_ {
	double linear_speed;
	double angular_speed;
};

coordinates goal_coord;
coordinates robot_coord;
robot_speed_ robot_speed;

nav_msgs::Odometry robot_orientation;
geometry_msgs::PointStamped closest_point;
visualization_msgs::Marker marker;


bool goto_closest(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	ROS_INFO("The service is running.");
	// IF THE flag_service IS 1 ROBOT STARTS MOVING
	flag_service = true;
	return true;
}

// FINDING COORDINATES OF CLOSEST OBJECT IN base_scan
void chatterCallbackScan (const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    double min_range = msg->ranges[0];
    int min_index = 0;
	
	for (int i = 0; i < msg->ranges.size(); i++)
	{
		if ((msg->ranges[i] > msg->range_min) && (msg->ranges[i] < msg->range_max)) {
			
			if (msg->ranges[i] < min_range) {
				min_range = msg->ranges[i];
				min_index = i;	
			}
			
		}
	}
    
    // CREATE NEW POINT AND ASSIGN IT COORDINATES OF THE CLOSEST OBJECT
    double angle = msg->angle_min + (min_index * msg-> angle_increment);
    
    closest_point.header.frame_id = "base_scan";
	closest_point.header.stamp = ros::Time();
		
    closest_point.point.x = min_range*cos(angle);
    closest_point.point.y = min_range*sin(angle);
    closest_point.point.z = 0.0;
    
}

// CONVERTING COORDINATES FROM base_scan TO odom
void transformPoint(const tf::TransformListener& listener)
{
    
	if (std::isinf(closest_point.point.x) || std::isnan(closest_point.point.y)) 
	{
		goal_coord.x = 0;
		goal_coord.y = 0;
		robot_speed.linear_speed = 0;
		robot_speed.angular_speed = 0;
	}
	else 
	{
		//IF GOAL COORDINATES ARE ASSIGNED INSIDE ROBOT'S FIELD OF VIEW
		flag = 1;
						
		geometry_msgs::PointStamped closest_point_odom;
		
		//INPUT TO FUNCTION:
		// 1 NAME OF THE LAYER WE WANT TO TRANSFORM THE POINT TO
		// 2 THE POINT WE'RE TRANSFORMING
		// 3 STORAGE FOR TRANSFORMED POINT
		listener.transformPoint("odom", closest_point, closest_point_odom);
		
		//ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
		//closest_point.point.x, closest_point.point.y, closest_point.point.z,
		//closest_point_odom.point.x, closest_point_odom.point.y, closest_point_odom.point.z, closest_point_odom.header.stamp.toSec());
	  
		goal_coord.x = closest_point_odom.point.x;
		goal_coord.y = closest_point_odom.point.y;

		// CHECK DATA OF ROBOT POSITION AND GOAL POSITION
		ROS_INFO("Goal position: x: [%f], y: [%f]", goal_coord.x, goal_coord.y);
		ROS_INFO("Robot position: x: [%f], y: [%f]", robot_coord.x, robot_coord.y);
	    	    	
	}

}


void chatterCallbackCurrent(const nav_msgs::Odometry::ConstPtr& msg)
{
    // GET COORDINATES OF ROBOT
    robot_coord.x = msg->pose.pose.position.x;
    robot_coord.y = msg->pose.pose.position.y;
    robot_orientation.pose.pose.orientation = msg->pose.pose.orientation;
}

void calculateRobotSpeed ()
{
  
  // GET ROBOT ANGLE AND GOAL ANGLE
  robot_coord.angle = tf::getYaw(robot_orientation.pose.pose.orientation);
  goal_coord.angle = -robot_coord.angle + atan2((goal_coord.y - robot_coord.y), (goal_coord.x - robot_coord.x));

  // DISTANCE TO GOAL
  distanceToGoal = sqrt((pow((goal_coord.x - robot_coord.x), 2)) + pow((goal_coord.y - robot_coord.y), 2));
  //ROS_INFO("Distance: %f", distanceToGoal);
 
  // CHOOSE THE SHORTEST PATH
  while(goal_coord.angle > M_PI) goal_coord.angle -= 2*M_PI;
  while(goal_coord.angle < -M_PI) goal_coord.angle += 2*M_PI;

  // ROTATION TO GOAL - ROTATION SPEED
  double KP = 2;
  robot_speed.angular_speed = KP * goal_coord.angle;
 
  // LIMIT ANGULAR SPEED
  if(robot_speed.angular_speed > 0.5)
    robot_speed.angular_speed = 0.5;
  else if(robot_speed.angular_speed < -0.5)
    robot_speed.angular_speed = -0.5;

 
  // LINEAR SPEED
  if(abs(goal_coord.angle) > 0.785398163) {
      robot_speed.linear_speed = 0;
  }
  else{
      robot_speed.linear_speed = std::min(0.1, distanceToGoal*0.2);
  } 
  
  //ROS_INFO("Angular speed: %f", robot_speed.angular_speed);
  //ROS_INFO("Linear speed: %f", robot_speed.linear_speed);    
}

// SETTING MARKER ON THE CLOSEST OBJECT
void markers()
{
	marker.header.frame_id = "odom";
	marker.header.stamp = ros::Time();
	marker.ns = "object";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = goal_coord.x;
	marker.pose.position.y = goal_coord.y;
	marker.pose.position.z = goal_coord.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.5;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zadaca3");
 
  ros::NodeHandle n;
 
  // SUBSCRIBE
  ros::Subscriber subCurrentPosition = n.subscribe("/odom", 1000, chatterCallbackCurrent);
  ros::Subscriber scanObjects = n.subscribe("/scan", 1000, chatterCallbackScan);

  // AUTOMATICALLY SUBSCRIBES TO THE TRANSFORM MESSAGE TOPIC AND MANAGES ALL TRANSFROM DATA
  tf::TransformListener listener;
 
  // TRANSFORM A POINT ONCE EVERY SECOND 
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener))); 
  
  // PUBLISH
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist msg;
  
  // PUBLISH MARKERS
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "/visualization_marker", 1);
  
  // REGISTER OUR SERVICE WITH THE MASTER
  ros::ServiceServer server = n.advertiseService("/goto_closest", &goto_closest);
  

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
     
      // IF THE FIRST GOAL POSITION IS ASSIGNED ENTER THE LOOP
      if(flag)
      {
		   
		if(flag_service)
		{
			// FUNCTION CALCULATE UPDATES THE GLOBAL VALUES OF LINEAR AND ANGULAR SPEED
			calculateRobotSpeed();
			
			if (distanceToGoal <= 0.3) 
			{
				robot_speed.linear_speed = 0;
				robot_speed.angular_speed = 0;
				flag_service = false;
			}
			
			msg.linear.x =  robot_speed.linear_speed;
			msg.angular.z = robot_speed.angular_speed;
			pub.publish(msg);
			
		}
		
        markers();
        vis_pub.publish(marker);
      }
      ros::spinOnce();
      loop_rate.sleep();
  }
 
  return 0;
}
