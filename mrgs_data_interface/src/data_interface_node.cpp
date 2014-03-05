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

/// ROS includes
#include "ros/ros.h"
#include "mrgs_data_interface/ForeignMap.h"
#include "mrgs_data_interface/ForeignMapVector.h"
#include "mrgs_data_interface/NetworkMap.h"

/// LZ4 include:
#include "lz4/lz4.h"

/// Wifi_comm includes
#include "wifi_comm/wifi_comm_lib.h"

/// Other includes
#include <string>
#include <fstream>

/// Global variables
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
  // Find the desired MAC's index
  int index = std::distance(g_peer_macs.begin(), std::find(g_peer_macs.begin(), g_peer_macs.end(), mac));
  // If the index isn't smaller than the size, the the desired MAC doesn't exist, 
  // and we return -1 to indicate just that.
  if(index < g_peer_macs.size())
    return index;
  else
    return -1;
}

void processForeignMap(std::string ip, const mrgs_data_interface::NetworkMap::ConstPtr& msg)
{
  /// Determine which robot sent the map (i.e. determine its ID) and act on that knowledge.
  int id = getRobotID(msg->mac);
  // Inform the outside world of our reception.
	ROS_INFO("Received a foreign map from %s, with mac %s, corresponding to id %d.", ip.c_str(), msg->mac.c_str(), id);
  if(id == -1)
  {
    // We've never found this robot before!
    // Add new robot to our list of peer macs
    id = g_peer_macs.size();          // The new MAC will be added at the end of the vector
    g_peer_macs.push_back(msg->mac);  // Add new MAC
  }
  
  // DEBUG: Print our current list of macs
  std::vector<std::string>::iterator i_mac_vector = g_peer_macs.begin();
  int i = 0;
  do
  {
  } while(i_mac_vector != g_peer_macs.end());
  
  /// Decompress data
  if(msg->decompressed_length > 0)  // Messages with this variable set to 0 are debug messages meant to test the network,
                                    // vector management, etc...
  {
    // Allocate and populate compressed buffer
    char* compressed = new char[msg->compressed_data.size()];
    for(int i = 0; i < msg->compressed_data.size(); i++)
      compressed[i] = msg->compressed_data.at(i);
    // Allocate decompression buffer
    char* decompressed = new char [msg->decompressed_length];
    // Decompress
    int decompressed_bytes = LZ4_decompress_safe(compressed, decompressed, msg->compressed_data.size(), msg->decompressed_length);
    
    /// Copy data to foreign map vector
    // id is used as the vector's index
  }
  else
    ROS_INFO("This is a debug map. No decompression took place.");
}

void newRobotInNetwork(char * ip)
{
  // Send
  g_my_comm->openForeignRelay(ip, "/external_map", true);
  // Receive
  char topic[128];
  ros::Subscriber sub = g_n->subscribe<mrgs_data_interface::NetworkMap>(wifi_comm::WiFiComm::concatTopicAndIp(topic, "/external_map", ip),
                                                                        1,  // Number of messages to keep on the input queue 
                                                                        boost::bind(processForeignMap, 
                                                                        std::string(ip), _1));
  subs.push_back(sub);
}

void processMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  // We simply change our global pointer to point to the latest map. The smart pointer should take care of deallocation.
  g_latest_local_map = map;
}

int main(int argc, char **argv)
{
  // Argument parsing
  if(argc < 2)
  {
    ROS_FATAL("I need the interface you want me to work with (must be the same olsrd is using)!");
    ROS_INFO("Usage: rosrun <package> <node> <interface>");
    return -1;
  }
  
  // ROS init
  ros::init(argc, argv, "data_interface_node");
  g_n = new ros::NodeHandle;
  
  // wifi_comm init
  g_my_comm = new wifi_comm::WiFiComm(newRobotInNetwork);
  ros::Publisher external_map = g_n->advertise<mrgs_data_interface::NetworkMap>("external_map", 10);
  
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
  
  
  // Declare callbacks
  ros::Subscriber map = g_n->subscribe<nav_msgs::OccupancyGrid>("map", 1, processMap);
  
  // Regular execution: loop with spinOnce
  ros::Rate r(1);
  mrgs_data_interface::NetworkMap msg;
  msg.mac = g_peer_macs.at(0);
  msg.decompressed_length = 0;  // Indicating this is a debug message
  while(ros::ok())
  {
    external_map.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
  

  return 0;
}
