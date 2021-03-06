/**
 * @file main.cpp
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

/**
 * @brief      main  Execution starts here
 *
 * @param      argc  The argc
 * @param      argv  The argv
 *
 * @return     Returns 0 upon successful execution
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  ros::NodeHandle n;
  Walker walk(n);
  return 0;
}
