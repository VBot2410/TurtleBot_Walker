/**
 * @file TurtleBot_Walker.cpp
 * @brief A simple program that drives turtlebot
 *        straight until it senses an obstacle and avoids it by turning. 
 *
 * @author Vaibhav Bhilare
 * @copyright 2017, Vaibhav Bhilare
 *
 * MIT License
 * Copyright (c) 2017 Vaibhav Bhilare
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* --Includes-- */
#include "TurtleBot_Walker.hpp"


void Walker::callback(const sensor_msgs::LaserScan::ConstPtr& Data_Input) {
  float Min_Dist = 0;
  for (int i = 0; i < Data_Input->ranges.size(); i++) {
    if (Data_Input->ranges[i] > Min_Dist)
      Min_Dist = Data_Input->ranges[i];
  }
  Obstacle_Distance = Min_Dist;
  ROS_INFO("Dist From Obstacle %0.2f", Obstacle_Distance);
}

Walker::Walker(ros::NodeHandle& n) {
  Laser_Subscribe = n.subscribe("/scan", 1000, &Walker::callback, this);
  pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  ros::Rate loop_rate(2);
  while (n.ok()) {
    //  Twist message is initiated
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    if (Obstacle_Distance > 0.75) {
      //  Linear motion in forward direction
      twist.linear.x = 0.1;
      ROS_INFO("Dist from Obstacle: %0.2f Moving Forward", Obstacle_Distance);
    } else {
      //  Rotation
      twist.angular.z = 1.5;
      ROS_INFO("Dist from Obstacle: %0.2f Turning", Obstacle_Distance);
    }
    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
