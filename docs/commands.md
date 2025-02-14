The command to get a list of frames (coordinate frames) in ROS 2 is:

Bash
ros2 topic echo /tf_static  # For static transforms
ros2 topic echo /tf       # For dynamic transforms

# This is to get the share directory of a particular package. 
ros2 pkg prefix nav2_bringup