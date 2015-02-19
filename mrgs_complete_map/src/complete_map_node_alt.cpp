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
 * complete_map_node
 *
 * Summary:
 * This node is responsible for processing a vector containing all the latest maps we've acquired from the local and
 * remote robots.
 * It receives a vector containing all the latest maps, even the repeated ones (so that the vector always contains the
 * latest map from each robot). Then it uses timestamps to determine which parts of the map tree need to be rebuilt, if
 * any.
 *
 * Methodology:
 *  1. Receive maps;
 *  2. Build a tree-like structure for storing and updating merged maps, and another for transforms;
 *  3. Send maps two by two to be aligned as needed. Merged maps and transforms are stored in their respective trees;
 *  4. Calculate and publish the complete_map to map transformations for each robot in the team;
 *  5. Publish the complete map.
 *
 */
// ROS includes
#include "ros/ros.h"
#include "mrgs_alignment/align.h"
#include "mrgs_data_interface/ForeignMapVector.h"
#include "mrgs_complete_map/LatestMapTF.h"
#include "geometry_msgs/TransformStamped.h"
#include <cstdlib>
#include "tf/transform_datatypes.h"

// Keeps the timestamps of the latest maps, so that when a new vector is received,
// only the necessary maps are re-aligned.
// To be edited only by the /foreign_maps callback
std::vector<ros::Time> g_latest_map_times;

// ROS facilities:
// To allow calls to service from callbacks
ros::ServiceClient g_client;
// To allow publishing from callbacks
ros::Publisher g_remote_tf_pub;
ros::Publisher g_foreign_map_pub;
ros::Publisher g_foreign_map_pub2;
ros::Publisher g_foreign_map_pub3;
std::vector<ros::Publisher> map_pub_vector;


void processForeignMaps(const mrgs_data_interface::ForeignMapVector::ConstPtr& maps, ros::NodeHandle n)
{
  /// Inform and start counting time
  ROS_INFO("Received a foreign map vector with %d maps.", maps->map_vector.size());
  ros::Time init = ros::Time::now();

  // Augment publisher vector, if needed
  if(maps->map_vector.size()-1 > map_pub_vector.size())
  {
    char topic_name[20];
    sprintf(topic_name, "/mrgs/robot_%d/map", map_pub_vector.size()+1);
    map_pub_vector.push_back(n.advertise<nav_msgs::OccupancyGrid>(topic_name, 1, true));
  }

  // Perform the iterative fusion and TF and map publication
  for(int i = 1; i < maps->map_vector.size(); i++)
  {
    // Fuse map i with 0 (regular call to the alignment node)
    mrgs_alignment::align srv;
    srv.request.map1 = maps->map_vector.at(0).map;
    srv.request.map2 = maps->map_vector.at(i).map;
    g_client.call(srv);

    // Publish TF
    mrgs_complete_map::LatestMapTF temp_latest;
    temp_latest.transform = srv.response.transform1;
    temp_latest.id = 0;
    g_remote_tf_pub.publish(temp_latest);
    temp_latest.transform = srv.response.transform2;
    temp_latest.id = i;
    g_remote_tf_pub.publish(temp_latest);

    // Publish map
    nav_msgs::OccupancyGrid temp_map;
    temp_map = maps->map_vector.at(i).map;    // Map i will be published
    char c_buffer[15];
    sprintf(c_buffer, "/robot_%d/map", i);
    temp_map.header.frame_id = c_buffer;
    map_pub_vector.at(i-1).publish(temp_map); // Publisher i-1 corresponds to map i.
  }

  /*
  if(maps->map_vector.size() == 2)
  {
    // Fuse the two maps and get the tfs
    mrgs_alignment::align srv;
    srv.request.map1 = maps->map_vector.at(0).map;
    srv.request.map2 = maps->map_vector.at(1).map;
    g_client.call(srv);

    // Publish tfs
    mrgs_complete_map::LatestMapTF temp_latest;
    temp_latest.transform = srv.response.transform1;
    temp_latest.id = 0;
    g_remote_tf_pub.publish(temp_latest);
    temp_latest.transform = srv.response.transform2;
    temp_latest.id = 1;
    g_remote_tf_pub.publish(temp_latest);

    // Publish foreign maps
    nav_msgs::OccupancyGrid temp_map;
    temp_map = maps->map_vector.at(1).map;
    temp_map.header.frame_id = "/robot_1/map";
    g_foreign_map_pub.publish(temp_map);
  }
  else if(maps->map_vector.size() == 3)
  {
    // Fuse the two maps and get the tfs
    mrgs_alignment::align srv;
    srv.request.map1 = maps->map_vector.at(0).map;
    srv.request.map2 = maps->map_vector.at(1).map;
    g_client.call(srv);

    // Publish tfs
    mrgs_complete_map::LatestMapTF temp_latest;
    temp_latest.transform = srv.response.transform1;
    temp_latest.id = 0;
    g_remote_tf_pub.publish(temp_latest);
    temp_latest.transform = srv.response.transform2;
    temp_latest.id = 1;
    g_remote_tf_pub.publish(temp_latest);

    // Fuse 2 into 0
    srv.request.map1 = maps->map_vector.at(0).map;
    srv.request.map2 = maps->map_vector.at(2).map;
    g_client.call(srv);

    // Publish tfs
    temp_latest.transform = srv.response.transform2;
    temp_latest.id = 2;
    g_remote_tf_pub.publish(temp_latest);

    // Publish foreign maps
    nav_msgs::OccupancyGrid temp_map;
    temp_map = maps->map_vector.at(1).map;
    temp_map.header.frame_id = "/robot_1/map";
    g_foreign_map_pub.publish(temp_map);
    temp_map = maps->map_vector.at(1).map;
    temp_map.header.frame_id = "/robot_2/map";
    g_foreign_map_pub2.publish(temp_map);
  }
  else if(maps->map_vector.size() == 4)
  {
    // Fuse the two maps and get the tfs
    mrgs_alignment::align srv;
    srv.request.map1 = maps->map_vector.at(0).map;
    srv.request.map2 = maps->map_vector.at(1).map;
    g_client.call(srv);

    // Publish tfs
    mrgs_complete_map::LatestMapTF temp_latest;
    temp_latest.transform = srv.response.transform1;
    temp_latest.id = 0;
    g_remote_tf_pub.publish(temp_latest);
    temp_latest.transform = srv.response.transform2;
    temp_latest.id = 1;
    g_remote_tf_pub.publish(temp_latest);

    // Fuse 2 into 0
    srv.request.map1 = maps->map_vector.at(0).map;
    srv.request.map2 = maps->map_vector.at(2).map;
    g_client.call(srv);

    // Publish tfs
    temp_latest.transform = srv.response.transform2;
    temp_latest.id = 2;
    g_remote_tf_pub.publish(temp_latest);

    // Fuse 3 into 0
    srv.request.map1 = maps->map_vector.at(0).map;
    srv.request.map2 = maps->map_vector.at(3).map;
    g_client.call(srv);

    // Publish tfs
    temp_latest.transform = srv.response.transform2;
    temp_latest.id = 3;
    g_remote_tf_pub.publish(temp_latest);

    // Publish foreign maps
    nav_msgs::OccupancyGrid temp_map;
    temp_map = maps->map_vector.at(1).map;
    temp_map.header.frame_id = "/robot_1/map";
    g_foreign_map_pub.publish(temp_map);
    temp_map = maps->map_vector.at(1).map;
    temp_map.header.frame_id = "/robot_2/map";
    g_foreign_map_pub2.publish(temp_map);
    temp_map = maps->map_vector.at(2).map;
    temp_map.header.frame_id = "/robot_3/map";
    g_foreign_map_pub2.publish(temp_map);

  }
  else
    ROS_FATAL("I'm not yet capable of dealing with %d maps!", maps->map_vector.size());*/

  // Inform
  ROS_INFO("Map vector processing took %fs.", (ros::Time::now() - init).toSec());
}

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "complete_map_node");
  ros::NodeHandle n;
  g_client = n.serviceClient<mrgs_alignment::align>("align");
  mrgs_alignment::align srv;
  //ros::Subscriber sub2 = n.subscribe("mrgs/foreign_maps", 1, processForeignMaps);
  g_remote_tf_pub = n.advertise<mrgs_complete_map::LatestMapTF>("mrgs/remote_tf", 10);
  //g_foreign_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/mrgs/robot_1/map", 10);
  //g_foreign_map_pub2 = n.advertise<nav_msgs::OccupancyGrid>("/mrgs/robot_2/map", 10);
  //g_foreign_map_pub3 = n.advertise<nav_msgs::OccupancyGrid>("/mrgs/robot_3/map", 10);
  ros::Subscriber sub2 = n.subscribe<mrgs_data_interface::ForeignMapVector>("mrgs/foreign_maps", 1, boost::bind(processForeignMaps, _1, n));

  // ROS loop
  ros::spin();

  // Never to be called
  return 0;
}
