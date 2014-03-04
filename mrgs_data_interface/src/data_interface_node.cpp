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
 * our local system. This node is also responsible for propagating our local maps throughout the network.
 * 
 * Methodology:
 */

// ROS includes
#include "ros/ros.h"
#include "mrgs_data_interface/ForeignMap.h"
#include "mrgs_data_interface/ForeignMapVector.h"
#include "mrgs_data_interface/NetworkMap.h"

// LZ4 include:
#include "lz4/lz4.h"

// Wifi_comm includes
#include "wifi_comm/wifi_comm_lib.h"

// Other includes
#include <string>
#include <fstream>

// Defines
//#define MRGS_INTERFACE "eth1"

// Global variables
// To be written only by the processMap callback
nav_msgs::OccupancyGrid::ConstPtr g_latest_local_map;
// To be written only by the processForeignMap callback and once in main()
// (at(0) is always our local mac)
std::vector<std::string> g_peer_macs;
// To be written by the processForeignMap callback
std::vector<mrgs_data_interface::ForeignMap::ConstPtr> g_foreign_map_vector;
// wifi_comm object
wifi_comm::WiFiComm* g_my_comm;
// Node handle. Must be global to be accessible by callbacks.
ros::NodeHandle * g_n;
// Global list of subscribers (also required by wifi_comm)
std::vector<ros::Subscriber> subs;

inline int getRobotID(std:: string mac){
  // We're in trouble if the desired mac doesn't exist.
  int i = 0;
  for(i = 0; i < g_peer_macs.size() && g_peer_macs.at(i) != mac; i++) {}
  if(i == g_peer_macs.size())
    return -1; // not found
  else
    return i;
}

void processForeignMap(std::string ip, const mrgs_data_interface::NetworkMap::ConstPtr& msg)
{
	ROS_INFO("Received a foreign map from %s.", ip.c_str());
  /// Determine which robot sent the map (i.e. determine its ID)
  /// (Add it to our list if we've never encountered it before)
  //int id = getRobotID(msg->mac);
  ROS_INFO("Received map has MAC = %s.", msg->mac.c_str());
  /*
  /// Decompress data
  // Allocate and populate compressed buffer
  char* compressed = new char[msg->compressed_data.size()];
  for(int i = 0; i < msg->compressed_data.size(); i++)
    compressed[i] = msg->compressed_data.at(i);
  // Allocate decompression buffer
  char* decompressed = new char [msg->decompressed_length];
  // Decompress
  int decompressed_bytes = LZ4_decompress_safe(compressed, decompressed, msg->compressed_data.size(), msg->decompressed_length);
  
  /// Copy data to foreign map vector
  // id is used as the vector's index*/
}

void newRobotInNetwork(char * ip)
{
  ROS_INFO("Setting up a new connection with %s.", ip);
  
  // Send
  g_my_comm->openForeignRelay(ip, "/external_map", true);

  // Receive
  char topic[128];
  ros::Subscriber sub = g_n->subscribe<mrgs_data_interface::NetworkMap>(wifi_comm::WiFiComm::concatTopicAndIp(topic, "/external_map", ip), 1, boost::bind(processForeignMap, std::string(ip), _1));
  subs.push_back(sub);
}

void processMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  g_latest_local_map = map;
}

int main(int argc, char **argv)
{
  // Argument parsing
  if(argc < 2)
  {
    ROS_FATAL("I need the interface you want me to work with!");
    return -1;
  }
  
  // ROS init
  ros::init(argc, argv, "data_interface_node");
  g_n = new ros::NodeHandle;
  
  // wifi_comm init
  g_my_comm = new wifi_comm::WiFiComm(newRobotInNetwork);
  ros::Publisher external_map = g_n->advertise<mrgs_data_interface::ForeignMap>("external_map", 10);
  
  // Retrieve local MAC address
  std::string* mac_file_path = new std::string(std::string("/sys/class/net/") + std::string(argv[1]) + std::string("/address"));
  std::string* local_mac = new std::string;
  std::ifstream mac_file;
  mac_file.open((*mac_file_path).c_str(), std::ios::in);
  if(mac_file.is_open())
  {
    mac_file >> *local_mac;
    mac_file.close();
    g_peer_macs.push_back(*local_mac);
    delete local_mac;
    delete mac_file_path;
  }
  else
  {
    ROS_ERROR("Can't open mac address file!");
    return -1;
  }
  
  ROS_INFO("Got mac: %s.", g_peer_macs.at(0).c_str());
  
  //ROS_INFO("Our local map is at index %d.", getRobotID(g_peer_macs.at(0)));
  
  // Declare callbacks
  ros::Subscriber map = g_n->subscribe<nav_msgs::OccupancyGrid>("map", 1, processMap);
  
  // Regular execution: loop with spinOnce
  ros::Rate r(1);
  mrgs_data_interface::NetworkMap msg;
  msg.mac = g_peer_macs.at(0);
  while(ros::ok())
  {
    ROS_INFO("Publishing message...");
    external_map.publish(msg);
    r.sleep();
  }
  

  return 0;
}
