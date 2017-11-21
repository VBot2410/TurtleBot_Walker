/**
 * @file TurtleBot_Walker.hpp
 * @brief A file containing function and variable delarations
 *         for the class Walker.
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

#ifndef CATKIN_WS_SRC_TURTLEBOT_WALKER_INCLUDE_TURTLEBOT_WALKER_HPP_
#define CATKIN_WS_SRC_TURTLEBOT_WALKER_INCLUDE_TURTLEBOT_WALKER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief      Walker class Declaration.
 * Declares various variables & functions in the class. 
 */
class Walker {
 public:  // Public Access Specifier
    /**
     * @brief      Constructs the object and 
     *             generates velocity commands
     *
     * @param      n     Nodehandle
     */
    explicit Walker(ros::NodeHandle& n);

    /**
     * @brief  callback    Callback for the subscriber
     * Checks the minimum distance from an obstacle
     *
     * @param  Data_Input  Message received over /scan topic
     */
    void callback(const sensor_msgs::LaserScan::ConstPtr& Data_Input);

 private:  // Private Access Specifier
    // Subscribe to the laserscan topic to get obstacles
    ros::Subscriber Laser_Subscribe;
    // Publish the Velocity topic to the TurtleBot
    ros::Publisher pub;
    geometry_msgs::Twist twist;
    // Varible to store Obstacle Distance
    float Obstacle_Distance;
    bool forward;
};
#endif  // CATKIN_WS_SRC_TURTLEBOT_WALKER_INCLUDE_TURTLEBOT_WALKER_HPP_
