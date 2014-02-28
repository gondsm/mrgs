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
 * data_interface_node:
 * 
 * Summary:
 * This node is responsible of keeping track of all the topics we must subscribe to in order to communicate with other
 * robots, as well as keeping local copies of foreign maps and propagating those maps in a standardized way across
 * our local system.
 * 
 * Methodology:
 */

// ROS includes
#include "ros/ros.h"
#include "mrgs_data_interface/ForeignMap.h"
#include "mrgs_data_interface/ForeignMapVector.h"

// Other includes
#include <string>
#include <fstream>

// Defines
#define MRGS_INTERFACE "eth0"

// Global variables
// To be written only once in main()
std::string g_local_mac;
// To be written only by the processMap callback
nav_msgs::OccupancyGrid::ConstPtr g_latest_local_map;

void processMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  g_latest_local_map = map;
}

int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "data_interface_node");
  ros::NodeHandle n;
  ros::Subscriber map = n.subscribe<nav_msgs::OccupancyGrid>("map", 1, processMap);
  
  // Retrieve local MAC address
  std::string mac_file_path = std::string("/sys/class/net/") + std::string(MRGS_INTERFACE) + std::string("/address");
  std::ifstream mac_file;
  mac_file.open(mac_file_path.c_str(), std::ios::in);
  if(mac_file.is_open())
  {
    mac_file >> g_local_mac;
    mac_file.close();
  }
  else 
    ROS_ERROR("Can't open file mac address file!");
  
  // Regular execution: loop with spinOnce
  ros::spin();

  return 0;
}
