#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>


struct coordinates {
	double x;
	double y;
	double z;
	double angle;
};

coordinates robot_coord;
coordinates ghost_coord;
double min_distance;
int ghost_num = 15;
bool found_ghosts = false;
bool go_to_ghost = false;
	

ros::Publisher pub;

// WHEN SERVICE /buster IS CALLED OR WHEN NEW GHOST APPEAR
void chatterCallbackGhost(const geometry_msgs::PolygonStamped::ConstPtr& ghosts)
{
	ROS_INFO("UPDATE");
	
	
	// GET NUMBER OF GHOSTS
    int ghost_num = ghosts->polygon.points.size();
    
    if (ghost_num > 0) {
		geometry_msgs::Point32 ghost = ghosts->polygon.points[0];
		
		// GET TEMPORARY FIRST GHOST AS CLOSEST
		int min_distance_index = 0;
		min_distance = sqrt((pow((ghost.x - robot_coord.x), 2)) + pow((ghost.y - robot_coord.y), 2));
		ghost = ghosts->polygon.points[0];
		ghost_coord.x = ghost.x;
		ghost_coord.y = ghost.y; 
		
		for (int i = 1; i < ghost_num; i++) 
		{
			ghost = ghosts->polygon.points[i];
			double distance_to_ghost = sqrt((pow((ghost.x - robot_coord.x), 2)) + pow((ghost.y - robot_coord.y), 2));
			
			if (distance_to_ghost < min_distance) {
				min_distance_index = i;
				min_distance = distance_to_ghost;
				
				// GET CLOSEST GHOST
				ghost_coord.x = ghost.x;
				ghost_coord.y = ghost.y;
		
			}
			
			ROS_INFO("Coordinate of ghost, x: %f, y: %f", ghost.x, ghost.y);
		}
		
		// FLAGS
		found_ghosts = true;
		go_to_ghost = true;
			
		// INFO OF THE CLOSEST GHOST
		ROS_INFO("coord, x: %lf, y: %lf", ghost_coord.x, ghost_coord.y);
		ROS_INFO("Distance to closest ghost: x: %lf", min_distance);
		ROS_INFO("WE HAVE GHOSTS : %d", ghost_num);   
		
	}
}

void chatterCallbackCurrent(const nav_msgs::Odometry::ConstPtr& msg)
{
    // GET COORDINATES OF ROBOT
    robot_coord.x = msg->pose.pose.position.x;
    robot_coord.y = msg->pose.pose.position.y;
    
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ghostbuster");
	ros::NodeHandle nh;
	
	// SUBSCRIBE
    ros::Subscriber subGoalPosition = nh.subscribe("/ghost", 1000, chatterCallbackGhost);
    ros::Subscriber subCurrentPosition = nh.subscribe("/odom", 1000, chatterCallbackCurrent);
 
    // PUBLISH
    pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
   
   
    ros::Rate loop_rate(20);
 
    while (ros::ok()){
		
		double distance = 0.5;
		if (found_ghosts) {
			distance = sqrt((pow((ghost_coord.x - robot_coord.x), 2)) + pow((ghost_coord.y - robot_coord.y), 2));
			ROS_INFO("Distance: %lf", distance);
		}
		
		if ((found_ghosts) && (distance < 0.5)) {
			std_srvs::Empty srv;
			ros::service::call("/buster", srv);
		} 
		else if (go_to_ghost) {
			geometry_msgs::PoseStamped msg2;
			msg2.header.seq = 1;
			msg2.header.frame_id = "map";
			msg2.header.stamp = ros::Time();
			msg2.pose.position.x = ghost_coord.x;
			msg2.pose.position.y = ghost_coord.y;
			msg2.pose.position.z = 0;
			msg2.pose.orientation.x = 0;
			msg2.pose.orientation.y = 0;
			msg2.pose.orientation.z = 0;
			msg2.pose.orientation.w = 1;
			pub.publish(msg2);
			
			go_to_ghost = false;
		}
	

		ros::spinOnce();
		loop_rate.sleep();
    }
   
    return 0;
 
}

