ros2 pkg create --build-type ament_cmake rb-car-sim


So far we have created a package with a launch script that can open gazebo and spawn the robot. 
We ported the xacro urdf robot from the old ros noetic repository to the current ros2 repo. Along with the xacro files are stl meshes that these files reference. Initially we had a problem where the robot did not load in gazebo properly and gazebo complained about not being able to find the stl meshes and began search them online. This was resolved by pointing the GAZEBO_MODELS_PATH to ros_ws/src. We added the following line to our dockerfile
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$HOME/ros_ws/src" >> ~/.bashrc which resolved the issue. 

Spawning the robot requires us to first convert the xacro file to urdf and providing the output as parameter to robot state publisher. 


## Controlling

We will be controlling the car spawned in gazebo. First we launch the gazebo spawn script. Then we will run a separate launch file to control the car using the keyboard. 

For controlling the car in gazebo we need a ackermann steering plugin. 


Package Dependencies (in package.xml):
Added ackermann_msgs - for Ackermann steering message types
Added ros2_control and ros2_controllers - for ROS 2 control framework
Added ackermann_steering_controller - for the actual steering controller


Gazebo Plugin Configuration (in rbcar_base.gazebo.xacro):
Removed the old differential drive configuration
Added new Ackermann steering macro with:
>
- Configured ROS 2 topics:
cmd_vel for velocity commands
odom for odometry
distance for distance measurements
Set up wheel joints and parameters:
Front and rear wheel joints
Steering joints
Wheel separation and diameter
Wheelbase
Added limits for:
Wheel torque (200 N⋅m)
Wheel acceleration (10.0 rad/s²)
Steering angle (0.6 rad ≈ 34.4°)








