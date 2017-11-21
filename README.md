# TurtleBot_Walker
Implementation of a simple walker algorithm on TurtleBot. </br ></br >
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Overview
ROS package containing a node to make a Turtlebot walk forward and turning when an obstacle is detected.</br >
## Dependencies
### ROS
ROS should be installed on the system. This package is tested on Ubuntu 16.04 LTS with [ROS Kinetic Distribution](http://wiki.ros.org/kinetic).<br />
Installation Instructions can be found [here](http://wiki.ros.org/kinetic/Installation).
### catkin
catkin is a Low-level build system macros and infrastructure for ROS.<br />
catkin is included by default when ROS is installed. It can also be installed with apt-get
```
sudo apt-get install ros-kinetic-catkin
```
### Package Dependency
- roscpp
- rospy
- sensor_msgs
- geometry_msgs
- turtlebot_gazebo
## Build Instructions
### Creating a catkin workspace
Create a catkin workspace using following instructions:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Running catkin_make command the first time in your workspace will create a CMakeLists.txt link in your 'src' folder. Before continuing source your new setup.*sh file:
```
$ source devel/setup.bash
```
### Building the Package inside catkin workspace
Clone the package in src folder of catkin workspace using following commands:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/VBot2410/TurtleBot_Walker.git
```
Then build the package using following commands:
```
$ cd ~/catkin_ws/
$ catkin_make
```
## Running The Demo
Open a terminal and run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch turtlebot_walker demo.launch
```
## Recording Bag files and how to Enable/Disable Recording:
Running the above command sets the record argument in launch file to *false* by default. To enable rebag recording, simply add **record:=true** to the launch command.</br >
The new commands will be:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
roslaunch turtlebot_walker demo.launch record:=true
```
This will generate a file named rosbag_recording.bag in results subdirectory.</br >
To disable the recording, use the default argument or specify **record:=false**.</br >
### Inspecting the bag file
For inspecting the recorded rosbag file, run following commands:
```
$ cd ~/catkin_ws/src/TurtleBot_Walker/results
$ rosbag info rosbag_recording.bag
```
This will produce an output similar to following:
```
path:        rosbag_recording.bag
version:     2.0
duration:    1:08s (68s)
start:       Dec 31 1969 19:00:00.46 (0.46)
end:         Dec 31 1969 19:01:09.39 (69.39)
size:        26.2 MB
messages:    51940
compression: none [34/34 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             kobuki_msgs/BumperEvent               [ffe360cd50f14f9251d9844083e72ac5]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            6916 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               6927 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              6929 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     6832 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                    194 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     138 msgs    : geometry_msgs/Twist                  
             /mobile_base/events/bumper                          12 msgs    : kobuki_msgs/BumperEvent              
             /mobile_base/sensors/imu_data                     6791 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  387 msgs    : bond/Status                           (3 connections)
             /odom                                             6803 msgs    : nav_msgs/Odometry                    
             /rosout                                            551 msgs    : rosgraph_msgs/Log                     (8 connections)
             /rosout_agg                                        533 msgs    : rosgraph_msgs/Log                    
             /scan                                              381 msgs    : sensor_msgs/LaserScan                
             /tf                                               8538 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage

```
### Playing Back the bag file
To play the recorded bag file, use following instructions:
In a terminal, type following command:
```
$ roscore
```
Open a new terminal and run following commands:
```
$ cd ~/catkin_ws/src/TurtleBot_Walker/results
$ rosbag play rosbag_recording.bag
```
Open a new terminal and run following command:
```
rqt_console
```
This will open a rqt_console which will play all the messages recorded in the bag file while recording.
