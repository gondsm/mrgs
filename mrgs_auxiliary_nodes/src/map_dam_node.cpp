/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo S. Martins, 2014
*********************************************************************/

/** 
 * map_dam_node
 * 
 * Summary:
 * This node is responsible for deciding whether or not to introduce a new local map into the internal network, in order
 * to avoid overloading it if a SLAM technique decides to publish maps very frequently.
 * 
 * Methodology:
 * The node intercepts the /map topic and publishes a new topic, which the data interface subscribes to.
 * 
 */
// ROS includes
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdlib>

// Global variables
ros::Publisher g_map_publisher;

void processUnfilteredMap(const nav_msgs::OccupancyGrid::ConstPtr& unfiltered_map)
{
  g_map_publisher.publish(unfiltered_map);
}

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "remote_map_node");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("map", 2, processUnfilteredMap);
  g_map_publisher = n.advertise<nav_msgs::OccupancyGrid>("mrgs/local_map", 2);
  // ROS loop
  ros::spin();

  // Never to be called
  return 0;
}
