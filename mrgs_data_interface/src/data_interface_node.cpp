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
 * Essentially, this node is charged with all the communication between robots, and with converting between datatypes 
 * used for external and internal communication.
 * This node is responsible for: keeping track of all the topics we must subscribe to in order to communicate with other
 * robots; keeping local copies of foreign maps transforms and poses; propagating those in a standardized way across
 * our local system; transmitting the latest local map, relevant transform and pose across the network.
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
 * I'm sure that with sufficient time, a few of these could be converted into local variables of some sort. However, in
 * order to complete this project in time, I have opted to combine this approach with programming discipline to ensure
 * there are no issues with using these variables. To any possible future maintaineres, austerity is advised in modify-
 * ing the way these variables interact with the remaining program.
 * 
 * Data Structures:
 * -> ForeignMap: A map received from another robot.
 * -> ForeignMapVector: A vector with a foreign map for each robot we know.
 * -> NetworkMap: The datatype that flows across the network. Contains a compressed map that is decompressed into a 
 * foreign map.
 * -> LatestRobotPose: A Pose including the robot's ID, for transmission into the internal network.
 * -> NetworkPose: The datatype that carries poses on the network. It is simply a pose with the sending robot's MAC
 * address attached.
 */

/// ROS includes
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_listener.h"

/// Include our messages
#include "mrgs_data_interface/ForeignMap.h"
#include "mrgs_data_interface/ForeignMapVector.h"
#include "mrgs_data_interface/NetworkMap.h"
#include "mrgs_data_interface/LatestRobotPose.h"
#include "mrgs_data_interface/NetworkPose.h"

/// LZ4 include:
#include "lz4/lz4.h"

/// Wifi_comm includes
#include "wifi_comm/wifi_comm_lib.h"

/// Other includes
#include <string>
#include <fstream>

/// Global variables
// To be written only by the processForeignMap callback (and once in main() for initialization)
// (at(0) is always our local mac)
std::vector<std::string> g_peer_macs;
// To be written by the processForeignMap callback
std::vector<mrgs_data_interface::ForeignMap> g_foreign_map_vector;
// wifi_comm object
wifi_comm::WiFiComm* g_my_comm;
// Node handle. Must be global to be accessible by callbacks.
ros::NodeHandle *g_n;
// Global list of subscribers (also required by wifi_comm)
std::vector<ros::Subscriber> g_subs;
// To be written only by the processMap callback!
mrgs_data_interface::NetworkMap::Ptr g_publish_map(new mrgs_data_interface::NetworkMap);
// To enable publishing from callback, to be edited once in main()
ros::Publisher g_foreign_map_vector_publisher;
// Publisher for external map (map that goes into the external network)
ros::Publisher *g_external_map;
// Publisher for poses from other robots
ros::Publisher g_latest_pose;
// Publisher for poses to other robots
ros::Publisher g_external_pose;
// Time at which the last pose transmission occurred
ros::Time g_since_last_pose;

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
  ROS_INFO("Received a foreign map from %s, with mac %s, id %d.", ip.c_str(), msg->mac.c_str(), id);
  bool is_repeated = false;
  if(id == -1)
  {
    // We've never found this robot before!
    // Add new robot to our list of peer macs and allocate space for its map.
    id = g_peer_macs.size();                      // The new MAC will be added at the end of the vector
    g_peer_macs.push_back(msg->mac);              // Add new MAC
    mrgs_data_interface::ForeignMap newMap;
    newMap.robot_id = id;                         // Attribute the right id
    g_foreign_map_vector.push_back(newMap);       // Add a new, uninitialized map.
    ROS_DEBUG("We've never met this guy before. His id is now %d. Vector sizes are %d and %d.", id, g_peer_macs.size(), g_foreign_map_vector.size());
  }
  else
  {
    // This is a robot we've met before. Let's see is we already have this map. We're not interested in re-decompressing
    // the same map.
    if(g_foreign_map_vector.at(id).map.header.stamp == msg->grid_stamp)
    {
      ROS_DEBUG("Repeated map, no need to decompress. Processing took %fs.", (ros::Time::now() - init).toSec());
      is_repeated = true;
    }
  }
  
  /// Decompress data
  if(msg->decompressed_length > 0 && is_repeated == false)  // Messages with decompressed_length == 0 are test messages.
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
    ROS_DEBUG("This is a debug or repeated map. No decompression took place.");
  
  /// Publish foreign maps
  // We only publish if the local map exists, so we don't send an empty map to the complete map node.
  if(g_publish_map->compressed_data.size() != 0)
  {
    ROS_DEBUG("Publishing foreign_map_vector...");
    mrgs_data_interface::ForeignMapVector map_vector;
    map_vector.map_vector = g_foreign_map_vector; // This is a potential time sink, depending on how the copy is handled.
    g_foreign_map_vector_publisher.publish(map_vector);
  }
  
  /// Inform
  ROS_INFO("Processing foreign map took %fs.", (ros::Time::now() - init).toSec());
}

void processNetworkPose(std::string ip, const mrgs_data_interface::NetworkPose::ConstPtr& new_pose)
{
  // Publish received pose in a LatestRobotPose message
  ROS_INFO("Received a new pose from the network.");
  int local_id = getRobotID(new_pose->mac);
  if(local_id == -1) return;  // We've never met this robot, we drop the message.
  mrgs_data_interface::LatestRobotPose latest_pose;
  latest_pose.transform = new_pose->transform;
  latest_pose.pose = new_pose->pose;
  latest_pose.id = local_id;
  g_latest_pose.publish(latest_pose);
}

void newRobotInNetwork(char * ip)
{
  // Inform
  ROS_INFO("Connecting to new peer at %s.", ip);
  // Send
  g_my_comm->openForeignRelay(ip, "/external_map", true);
  g_my_comm->openForeignRelay(ip, "/external_pose", true);
  //char topic1[128];
  //g_my_comm->openForeignRelay(ip, "/external_map", wifi_comm::WiFiComm::concatTopicAndIp(topic1, "/external_map", ip));
  // Receive
  char topic[128];
  ros::Subscriber sub = g_n->subscribe<mrgs_data_interface::NetworkMap>(wifi_comm::WiFiComm::concatTopicAndIp(topic, "/external_map", ip),
                                                                        1,  // Number of messages to keep on the input queue 
                                                                        boost::bind(processForeignMap, 
                                                                        std::string(ip), _1));
  ros::Subscriber sub2 = g_n->subscribe<mrgs_data_interface::NetworkPose>(wifi_comm::WiFiComm::concatTopicAndIp(topic, "/external_pose", ip),
                                                                        3,  // Number of messages to keep on the input queue 
                                                                        boost::bind(processNetworkPose, 
                                                                        std::string(ip), _1));                                                                        
  g_subs.push_back(sub);
  g_subs.push_back(sub2);
}

void processMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  // This function processes a new local map. It updates the latest local map pointer and creates a new publish-able
  // NetworkMap.
  
  // Start counting time
  ros::Time init = ros::Time::now();
  
  /// Update the local map
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
  char* compressed = new char [LZ4_compressBound(map_length)];              // We have to allocate this buffer with 
  char* decompressed = new char [map_length];                               // extra space, lest the data be 
  for(int i = 0; i < map_length; i++)                                        // incompressible.
    decompressed[i] = map->data.at(i);                                        // Copy data to compress.
  int compressed_bytes = LZ4_compress(decompressed, compressed, map_length);  // Compress
  // Store the new map
  g_publish_map->compressed_data.clear();
  g_publish_map->compressed_data.reserve(compressed_bytes);
  for(int i = 0; i < compressed_bytes; i++)
    g_publish_map->compressed_data.push_back(compressed[i]);
  // Publish
  g_external_map->publish(*g_publish_map);
  
  /// Inform
  ROS_INFO("Processed a new local map. Size: %d bytes. Compressed size: %d bytes. Ratio: %f", 
           map_length, compressed_bytes, (float)map_length/(float)compressed_bytes);
  ROS_INFO("Processing local map took %fs.", (ros::Time::now() - init).toSec());
}

void processOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
  // Check how long ago we've published a pose into the network. If sufficient time has passed, we publish a new one.
  // We'll use 5 seconds as a base time between pose transmission.
  if(ros::Time::now() - g_since_last_pose > ros::Duration(5.0))
  {
    // Check if transform exists. If not, we return
    tf::TransformListener tf_listener;
    if(!tf_listener.canTransform("/odom", "/map", ros::Time::now()))
    {
      ROS_INFO("Could not obtain transform, won't publish new pose.");
      return;
    }
    
    // Write data to new message
    mrgs_data_interface::NetworkPose new_pose;
    new_pose.mac = g_peer_macs.at(0);
    new_pose.pose = odom->pose.pose;
    tf::StampedTransform new_tf;
    tf_listener.lookupTransform("/odom", "/map", ros::Time::now(), new_tf);
    tf::transformStampedTFToMsg(new_tf, new_pose.transform);
    
    // Publish
    ROS_INFO("Publishing new pose into network.");
    g_external_pose.publish(new_pose);
  }
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
  boost::function<void (char *)> new_robot_callback;
  new_robot_callback = newRobotInNetwork;
  g_my_comm = new wifi_comm::WiFiComm(new_robot_callback);
  g_external_map = new ros::Publisher;
  *g_external_map = g_n->advertise<mrgs_data_interface::NetworkMap>("external_map", 10);
  g_foreign_map_vector_publisher = g_n->advertise<mrgs_data_interface::ForeignMapVector>("foreign_maps", 10);
  g_latest_pose = g_n->advertise<mrgs_data_interface::LatestRobotPose>("remote_nav/remote_poses", 10);
  g_since_last_pose = ros::Time::now();
  g_external_pose = g_n->advertise<mrgs_data_interface::NetworkPose>("external_pose", 10);
  
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
  ros::Subscriber odom = g_n->subscribe<nav_msgs::Odometry>("odom", 1, processOdom);
  
  // Regular execution:
  ros::spin();
  

  return 0;
}
