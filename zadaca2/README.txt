# MOBILE ROBOTICS - LOCALIZATION AND LOCOMOTION

## PROGRAM DESCRIPTION
Robot goes to goal position - position set through GUI of Rviz

### WHAT PROGRAM CONTAINS
Get coordinates of robot position
Subscribe to topic "/odom"
Get coordinates of goal position set through GUI of Rviz
Subscribe to topic "/move_base_simple/goal"
Move robot to goal position
Publish to topic "/cmd_vel"


## HOW TO RUN PROGRAM
1. Type "roslaunch zadaca2 zadaca2.launch" in terminal
2. Set 2D Nav Goal through GUI of Rviz