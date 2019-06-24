## Mobile robotics - Planning and Navigation

### Program description
Robot looks for ghosts and when ghost is inside radius of 0.5 meters service /buster is called.
Robot continues that process until he catches all the ghosts.

### What program contains
Subscribe to topic /ghost
Get ghosts in type geometry_msgs::PolygonStamped
Find closest ghost
Go to that position
Repeat

### How to run program
Type "roslaunch ghostbuster sve.launch" in terminal