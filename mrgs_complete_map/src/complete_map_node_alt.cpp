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

// ROS facilities:
// To allow calls to service from callbacks
ros::ServiceClient g_client;
// To allow publishing from callbacks
ros::Publisher g_remote_tf_pub;
ros::Publisher g_foreign_map_pub;


void processForeignMaps(const mrgs_data_interface::ForeignMapVector::ConstPtr& maps)
{
  /// Inform and start counting time
  ROS_INFO("Received a foreign map vector with %d maps.", maps->map_vector.size());
  ros::Time init = ros::Time::now();

  // Fuse the two maps and get the tfs
  mrgs_alignment::align srv;
  srv.request.map1 = maps->map_vector.at(0).map;
  srv.request.map2 = maps->map_vector.at(1).map;

  // Publish the maps and tfs
  mrgs_complete_map::LatestMapTF temp_latest;
  temp_latest.transform = srv.response.transform1;
  temp_latest.id = 0;
  g_remote_tf_pub.publish(temp_latest);
  temp_latest.transform = srv.response.transform2;
  temp_latest.id = 1;
  g_remote_tf_pub.publish(temp_latest);

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
  ros::Subscriber sub2 = n.subscribe("mrgs/foreign_maps", 1, processForeignMaps);
  g_remote_tf_pub = n.advertise<mrgs_complete_map::LatestMapTF>("mrgs/remote_tf", 10);
  g_foreign_map_pub = n.advertise<nav_msgs::OccupancyGrid>("robot_1/map", 10);;

  // ROS loop
  ros::spin();

  // Never to be called
  return 0;
}
