# MOBILE ROBOTICS - LOCALIZATION AND LOCOMOTION

## PROGRAM DESCRIPTION
Robot goes to the closest object which is detected by sensor

### WHAT PROGRAM CONTAINS
Get coordinates of robot position
Subscribe to topic "/odom"
Get coordinates of the closest object set through GUI of Rviz
Subscribe to topic "/scan"
Move robot to closest object
Publish to topic "/cmd_vel"
Service goto_closest
Publish to topic "/visualization_marker"

This program was run on real mobile robot TURTLEBOT3 Burger.

## HOW TO RUN PROGRAM
1. Type "roslaunch zadaca3 zadaca3.launch" in terminal
2. Set closest object with 2D Nav Goal through GUI of Rviz
3. Type "rosservice call \goto_closest" in terminal