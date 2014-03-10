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
 * This node maintains a multitude of global variables that, together, represent the current state of the operation, as
 * seen by the local robot.
 * -> g_latest_local_map: The latest map published in /map.
 * -> g_peer_macs: A list of robots we have already encountered.
 * -> g_foreign_map_vector: A vector of foreign maps, for publishing in the local network.
 * -> g_my_comm: WiFiComm object, responsible for facilitating communication.
 * -> g_n: ROS node handle, must be global to make subscription of topics in callbacks possible.
 * -> g_subs: Vector os subscriptions.
 * -> g_publish_map: An external map, for publishing in the network connecting multiple robots.
 * These variables are only written to on very very well determined moments, to prevent race conditions and other data-
 * -related issues.
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
//nav_msgs::OccupancyGrid::ConstPtr g_latest_local_map;
// To be written only by the processForeignMap callback and once in main()
// (at(0) is always our local mac)
std::vector<std::string> g_peer_macs;
// To be written by the processForeignMap callback
//std::vector<mrgs_data_interface::ForeignMap::Ptr> g_foreign_map_vector;
std::vector<mrgs_data_interface::ForeignMap> g_foreign_map_vector;
// wifi_comm object
wifi_comm::WiFiComm* g_my_comm;
// Node handle. Must be global to be accessible by callbacks.
ros::NodeHandle * g_n;
// Global list of subscribers (also required by wifi_comm)
std::vector<ros::Subscriber> subs;
// To be written only by the processMap callback!
mrgs_data_interface::NetworkMap::Ptr g_publish_map(new mrgs_data_interface::NetworkMap);
// To enable publishing from callback, to be edited once in main()
ros::Publisher g_foreign_map_vector_publisher;

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
  // Start counting time
  ros::Time init = ros::Time::now();
  
  /// Determine which robot sent the map (i.e. determine its ID) and act on that knowledge.
  int id = getRobotID(msg->mac);
  // Inform the outside world of our reception.
  ROS_INFO("Received a foreign map from %s, with mac %s, corresponding to id %d.", ip.c_str(), msg->mac.c_str(), id);
  if(id == -1)
  {
    // We've never found this robot before!
    // Add new robot to our list of peer macs and allocate space for its map.
    id = g_peer_macs.size();                       // The new MAC will be added at the end of the vector
    g_peer_macs.push_back(msg->mac);               // Add new MAC
    mrgs_data_interface::ForeignMap newMap;
    newMap.robot_id = id;                         // Attribute the right id
    g_foreign_map_vector.push_back(newMap);        // Add a new, uninitialized map.
    ROS_DEBUG("We've never met this guy before. His id is now %d. Vector sizes are %d and %d.", id, g_peer_macs.size(), g_foreign_map_vector.size());
  }
  else
  {
    // This is a robot we've met before. Let's see is we already have this map. We're not interested in re-decompressing
    // the same map.
    if(g_foreign_map_vector.at(id).map.header.stamp == msg->grid_stamp)
    {
      ROS_INFO("Repeated map, no need to decompress. Processing took %fs.", (ros::Time::now() - init).toSec());
      return;
    }
  }
  
  
  /// Decompress data
  if(msg->decompressed_length > 0)  // Messages with this variable set to 0 are debug messages meant to test the network,
                                    // vector management, etc...
  {
    ROS_DEBUG("Received map consists of %d compressed bytes. Decompressing.", msg->compressed_data.size());
    // Allocate and populate compressed buffer
    char* compressed = new char[msg->compressed_data.size()];
    for(int i = 0; i < msg->compressed_data.size(); i++)
      compressed[i] = msg->compressed_data.at(i);
    // Allocate decompression buffer
    char* decompressed = new char [msg->decompressed_length];
    // Decompress
    int decompressed_bytes = LZ4_decompress_safe(compressed, decompressed, msg->compressed_data.size(), msg->decompressed_length);
    
    /// Copy data to foreign map vector
    // Copy metadata
    g_foreign_map_vector.at(id).map.header.stamp = msg->grid_stamp;
    g_foreign_map_vector.at(id).map.info = msg->info;
    g_foreign_map_vector.at(id).map.data.clear();
    // Pre-allocate and copy map
    g_foreign_map_vector.at(id).map.data.reserve(decompressed_bytes);
    for(int i = 0; i < decompressed_bytes; i++)
      g_foreign_map_vector.at(id).map.data.push_back(decompressed[i]);
  }
  else
    ROS_DEBUG("This is a debug map. No decompression took place.");
  
  /// Publish foreign maps
  ROS_DEBUG("Publishing foreign_map_vector...");
  mrgs_data_interface::ForeignMapVector map_vector;
  map_vector.map_vector = g_foreign_map_vector;
  g_foreign_map_vector_publisher.publish(map_vector);
  
  /// Inform
  ROS_INFO("Processing foreign map took %fs.", (ros::Time::now() - init).toSec());
}

void newRobotInNetwork(char * ip)
{
  // Inform
  ROS_DEBUG("Connecting to new peer at %s.", ip);
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
  // This function processes a new local map. It updates the latest local map pointer and creates a new publish-able
  // NetworkMap.
  
  // Start counting time
  ros::Time init = ros::Time::now();
  
  /// Update the local map
  //g_latest_local_map = map;
  g_foreign_map_vector.at(0).map = *map;
  
  /// Create the new NetworkMap
  // Fill in local mac
  g_publish_map->mac = g_peer_macs.at(0);
  // Indicate that it's a complete map
  g_publish_map->is_complete = true;
  // Fill in time stamp, metadata and decompressed length
  g_publish_map->grid_stamp = map->header.stamp;
  g_publish_map->info = map->info;
  unsigned int map_length = g_publish_map-> info.height * g_publish_map-> info.width;
  g_publish_map->decompressed_length = map_length;
  // Compress the new map
  char* compressed = new char [LZ4_compressBound(map_length)];                // We have to allocate this buffer with 
  char* decompressed = new char [map_length];                                 // extra space, lest the data be 
  for(int i = 0; i < map_length; i++)                                         // incompressible.
    decompressed[i] = map->data.at(i);                                        // Copy data to compress.
  int compressed_bytes = LZ4_compress(decompressed, compressed, map_length);  // Compress
  // Store the new map
  g_publish_map->compressed_data.clear();
  g_publish_map->compressed_data.reserve(compressed_bytes);
  for(int i = 0; i < compressed_bytes; i++)
    g_publish_map->compressed_data.push_back(compressed[i]);
    
  /// Inform
  ROS_INFO("Processed a new local map. Size: %d bytes. Compressed size: %d bytes. Ratio: %f", 
           map_length, compressed_bytes, (float)map_length/(float)compressed_bytes);
  ROS_INFO("Processing local map took %fs.", (ros::Time::now() - init).toSec());
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
  g_foreign_map_vector_publisher = g_n->advertise<mrgs_data_interface::ForeignMapVector>("foreign_maps", 10);
  
  // Retrieve local MAC address
  std::string* mac_file_path = new std::string(std::string("/sys/class/net/") + 
                                               std::string(argv[1]) + 
                                               std::string("/address"));
  std::string* local_mac = new std::string;
  std::ifstream mac_file;
  mac_file.open((*mac_file_path).c_str(), std::ios::in);
  if(mac_file.is_open())
  {
    mac_file >> *local_mac;
    mac_file.close();
    g_peer_macs.push_back(*local_mac);
    delete local_mac;                   // No need to keep this extra stuff in memory
    delete mac_file_path;               // Idem
  }
  else
  {
    ROS_FATAL("Can't open mac address file, terminating.");
    return -1;
  }
  
  // Push an empty map into the foreign map vector, to keep it aligned with IDs.
  mrgs_data_interface::ForeignMap emptyMap;
  emptyMap.robot_id = 0;
  g_foreign_map_vector.push_back(emptyMap);
  
  // Declare callbacks
  ros::Subscriber map = g_n->subscribe<nav_msgs::OccupancyGrid>("map", 1, processMap);
  
  // Regular execution: loop with spinOnce
  ros::Rate r(1);
  while(ros::ok())
  {
    // Publish external map
    if(g_publish_map->compressed_data.size() == 0) // Only publish if the local, compressed, map exists.
      ROS_DEBUG("No map to publish yet.");
    else
    {
      ROS_DEBUG("Publishing map...");
      external_map.publish(*g_publish_map);
    }
    
    // Spin and sleep
    ros::spinOnce();
    r.sleep();
  }
  

  return 0;
}
