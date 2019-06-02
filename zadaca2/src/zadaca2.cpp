#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include <math.h>

using namespace std;

nav_msgs::Odometry robot_orientation;

double robot_x;
double robot_y;
double goal_x;
double goal_y;

double robot_angle;
double goal_angle;

double distanceToGoal;

double linear_speed;
double angular_speed;

bool flag = false;


void chatterCallbackGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //SET FLAG TO 1 WHEN THE FIRST GOAL IS ASSIGNED
  flag = true;
 
  //GET COORDINATES OF GOAL
  goal_x = msg->pose.position.x;
  goal_y = msg->pose.position.y;
 
}

void chatterCallbackCurrent(const nav_msgs::Odometry::ConstPtr& msg)
{
      
    // GET COORDINATES OF ROBOT
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_orientation.pose.pose.orientation = msg->pose.pose.orientation;
 
}

void calculateRobotSpeed();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zadaca2");

  ros::NodeHandle n;
 
  // SUBSCRIBE
  ros::Subscriber subGoalPosition = n.subscribe("/move_base_simple/goal", 1000, chatterCallbackGoal);
  ros::Subscriber subCurrentPosition = n.subscribe("/odom", 1000, chatterCallbackCurrent);
  
  // PUBLISH
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist msg;
 
  ros::Rate loop_rate(20);
 
  while (ros::ok()){
     
      //IF THE FIRST GOAL POSITION IS ASSIGNED ENTER THE LOOP
      if(flag) {   
        //FUNCTION CALCULATE UPDATES THE GLOBAL VALUES OF LINEAR AND ANGULAR SPEED 
        calculateRobotSpeed();
        msg.linear.x =  linear_speed;
        msg.angular.z = angular_speed;
        pub.publish(msg);
      }
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}

// IN WHICH DIRECTION ROBOT NEEDS TO GO TO COME TO GOAL (LINEAR SPEED - GO STRAIGHT, ANGULAR SPEED - TURN AROUND)
void calculateRobotSpeed ()
{
  // CHECK DATA OF ROBOT POSITION AND GOAL POSITION
  ROS_INFO("Goal position: x: [%f], y: [%f]", goal_x, goal_y);
  ROS_INFO("Robot position: x: [%f], y: [%f]", robot_x, robot_y);
 
  // GET ROBOT ANGLE AND GOAL ANGLE (GOAL ANGLE IS 0 WHEN ROBOT IS LOOKING STRAIGHT AT GOAL)
  robot_angle = tf::getYaw(robot_orientation.pose.pose.orientation);
  goal_angle = -robot_angle + atan2((goal_y - robot_y), (goal_x - robot_x));
 
  // CHECK DATA OF ROBOT ANGLE AND GOAL ANGLE
  ROS_INFO("Angle of robot: %f", robot_angle); 
  ROS_INFO("Angle of goal: %f", goal_angle);
 
  // DISTANCE TO GOAL
  distanceToGoal = sqrt((pow((goal_x - robot_x), 2)) + pow((goal_y - robot_y), 2));
  ROS_INFO("Distance: %f", distanceToGoal);
 
  // CHOOSE THE SHORTEST PATH
  while(goal_angle > M_PI) goal_angle -= 2*M_PI;
  while(goal_angle < -M_PI) goal_angle += 2*M_PI;

  // ROTATION TO GOAL - ROTATION SPEED
  double KP = 2;
  angular_speed = KP * goal_angle;
 
  // LIMIT ANGULAR SPEED
  if(angular_speed > 0.8)
    angular_speed = 0.8;
  else if(angular_speed < -0.8)
    angular_speed = -0.8;
   
  ROS_INFO("Angular speed: %f", angular_speed);
 
  // LINEAR SPEED IS 0 WHEN GOAL ANGLE IS MORE THAN 90
  if(abs(goal_angle) > 0.785398163) {
      linear_speed = 0;
  }
  else{
      linear_speed = min(0.2, distanceToGoal*0.5);
  }
 
  ROS_INFO("Linear speed: %f", linear_speed);
      
}
