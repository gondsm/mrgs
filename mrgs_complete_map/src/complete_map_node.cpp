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
 * 
 * Methodology:
 *  1. Receive maps
 *  3. Send them two by two to be aligned
 *  4. Publish the complete map
 * 
 */
// ROS includes
#include "ros/ros.h"
#include "mrgs_alignment/align.h"
#include "mrgs_data_interface/ForeignMapVector.h"
#include "mrgs_complete_map/LatestMapTF.h"
#include <cstdlib>

// Global variables
// To be edited only by the /foreign_maps callback
std::vector<ros::Time> g_latest_map_times;
// To be edited only by the processForeignMaps callback
// Keeps the "dirtiness" (the need to be rebuilt) of aligned maps.
std::vector<std::vector<bool> > g_is_dirty;
// To be edited by the processForeignMaps callback
std::vector<std::vector<nav_msgs::OccupancyGrid> > g_aligned_maps;
std::vector<std::vector<geometry_msgs::TransformStamped> > g_transforms;
// To allow calls to service from callbacks
ros::ServiceClient g_client;
// To allow publishing from callbacks
ros::Publisher pub1;
ros::Publisher pub2;

geometry_msgs::Quaternion multiplyQuaternion(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
  geometry_msgs::Quaternion q3;
  
  return q3;
}

void processForeignMaps(const mrgs_data_interface::ForeignMapVector::ConstPtr& maps)
{
  /// Inform and start counting time
  ROS_INFO("Received a foreign map vector with %d maps.", maps->map_vector.size());
  g_is_dirty.clear(); // This variable will later become local.
  ros::Time init = ros::Time::now();
  
  /// Allocate dirtiness matrix
  ROS_DEBUG("Allocating dirtiness matrix...");
  // Allocate first row
  g_is_dirty.push_back(std::vector<bool>(maps->map_vector.size(), true));
  // Allocate subsequent rows. We do this row by row until the las row only has one element.
  ROS_DEBUG("First row allocated with %d elements. Allocating others...", maps->map_vector.size());
  int i = 0;
  do
  {
    i++;
    int prev_n = g_is_dirty.at(i-1).size();
    int curr_n;
    prev_n % 2 == 0? curr_n = prev_n/2:curr_n = (prev_n+1)/2;
    g_is_dirty.push_back(std::vector<bool>(curr_n, true));
    ROS_DEBUG("Allocated a new row with %d elements.", curr_n);
  }while(g_is_dirty.at(i).size() > 1);
  ROS_DEBUG("Allocated %d new rows.", i);
  
  /// Expand aligned map matrix (if needed)
  // This methodology is a bit wasteful, but will rarely be used, so it's passable.
  ROS_DEBUG("Allocating map matrix.");
  std::vector<nav_msgs::OccupancyGrid> empty_vec;
  nav_msgs::OccupancyGrid empty_map;
  geometry_msgs::TransformStamped empty_transform;
  while(g_aligned_maps.size() < g_is_dirty.size()-1)
  {
    g_aligned_maps.push_back(empty_vec);
  }
  for(int i = 0; i < g_is_dirty.size()-1; i++)
  {
    while(g_aligned_maps.at(i).size() < g_is_dirty.at(i+1).size())
    {
      ROS_DEBUG("Pushing empty map into line %d of the aligned maps vector.", i);
      g_aligned_maps.at(i).push_back(empty_map);
      g_transforms.at(i).push_back(empty_transform);
    }
  }
  
  /// Check if the received maps have updates and mark as dirty accordingly
  ROS_DEBUG("Marking updates.");
  if(g_latest_map_times.size()==0)
  {
    // There is no previous data, we're on the first run: we fill in the vector with the times we've received.
    ROS_DEBUG("First run, all maps are dirty");
    for(int i = 0; i < maps->map_vector.size(); i++)
    {
      g_latest_map_times.push_back(maps->map_vector.at(i).map.header.stamp);
    }
  }
  else
  {
    // If this isn't the first run, we check the maps we already have and mark them dirty as needed. If the vector we
    // received is bigged than the last, all those new maps are marked as dirty, in order to trigger the building of
    // their side of the tree.
    for(int i = 0; i < g_is_dirty.at(0).size(); i++)
    {
      if(i >= g_latest_map_times.size() || g_latest_map_times.at(i) < maps->map_vector.at(i).map.header.stamp)
      {
        g_is_dirty.at(0).at(i) = true;
        ROS_DEBUG("Map %d is dirty.", i);
      }
      else
        g_is_dirty.at(0).at(i) = false;
    }
  }
  
  /// Renew our latest map times
  ROS_DEBUG("Updating map times.");
  g_latest_map_times.clear();
  g_latest_map_times.reserve(maps->map_vector.size());
  for(int i = 0; i < maps->map_vector.size(); i++)
    g_latest_map_times.push_back(maps->map_vector.at(i).map.header.stamp);

  /// (Re-)Build maps
  // Iterate through the dirtiness matrix, starting in row 1 (not 0), and rebuild 
  // all maps which depend on a "dirty" map. Newly-built maps are also marked as "dirty".
  // This process is repeated until we only have a single map to build, which will be our final map.
  ROS_DEBUG("Rebuilding necessary maps.");
  for(int i = 1;i < g_is_dirty.size(); i++)
  {
    for(int j = 0; j < g_is_dirty.at(i).size(); j++)
    {
      // How many maps does this one depend on?
      if(2*j == g_is_dirty.at(i-1).size() - 1)
      {
        // This map only depends on one other, so we simply pull it up
        if(i == 1)
        {
          // First row, we must pull from the newly-received maps
          g_aligned_maps.at(i-1).at(j) = maps->map_vector.at(2*j).map;
        }
        else
        {
          // Subsequent rows, we pull from the row beneath
          g_aligned_maps.at(i-1).at(j) = g_aligned_maps.at(i-2).at(2*j);
        }
      }
      else
      {
        // This map depends on two others.
        // Are any of them dirty?
        if(g_is_dirty.at(i-1).at(2*j) || g_is_dirty.at(i-1).at((2*j)+1))
        {
          // Then the current map is dirty
          ROS_DEBUG("Dirty map at i = %d and j = %d.", i, j);
          g_is_dirty.at(i).at(j) = true;
          // ... and we must reallign it
          mrgs_alignment::align srv;
          if(i == 1)
          {
            // We're on the first line, the source for maps is the vector we received
            ROS_DEBUG("Merging from the first line.");
            srv.request.map1 = maps->map_vector.at(2*j).map;
            srv.request.map2 = maps->map_vector.at((2*j)+1).map;
            srv.request.crop = false;
          }
          else
          {
            ROS_DEBUG("Merging from a not-first line.");
            srv.request.map1 = g_aligned_maps.at(i-2).at(2*j);
            srv.request.map2 = g_aligned_maps.at(i-2).at((2*j)+1);
            srv.request.crop = false;
          }
          ROS_DEBUG("Sending request...");
          if(!g_client.call(srv)) 
            ROS_FATAL("Error calling service!");
          else
          {
            ROS_DEBUG("Got a response! Adding map to our matrix.");
            g_aligned_maps.at(i-1).at(j) = srv.response.merged_map;
            g_transforms.at(i-1).at(2*j) = srv.response.transform1;
            g_transforms.at(i-1).at((2*j)+1) = srv.response.transform2;
            ROS_WARN("Received index: %f", srv.response.success_coefficient);
          }
        }
        else
        {
          // If not, we mark the current map as clean and proceed
          g_is_dirty.at(i).at(j) = false;
        }
      }
    }
  }
  
  /// Recaltulate and publish all our complete_map to map transformations
  // Iterate through every map and calculate its transform
  for(int i = 0; i < g_latest_map_times.size(); i++)
  {
    // Calculate transform
    geometry_msgs::TransformStamped temp_transform;
    char buffer[10];
    sprintf(buffer, "%d", i);
    temp_transform.header.frame_id = std::string(std::string("robot_") + std::string(buffer) + std::string("/map"));
    temp_transform.child_frame_id = "complete_map";
    temp_transform.header.stamp = ros::Time::now();
    temp_transform.transform.translation.x = g_transforms.at(0).at(i).transform.translation.x;
    temp_transform.transform.translation.y = g_transforms.at(0).at(i).transform.translation.y;
    temp_transform.transform.translation.z = g_transforms.at(0).at(i).transform.translation.z;
    temp_transform.transform.rotation.x = g_transforms.at(0).at(i).transform.rotation.x;
    temp_transform.transform.rotation.y = g_transforms.at(0).at(i).transform.rotation.y;
    temp_transform.transform.rotation.z = g_transforms.at(0).at(i).transform.rotation.z;
    temp_transform.transform.rotation.w = g_transforms.at(0).at(i).transform.rotation.w;
    for(int j = 1; j < g_transforms.size(); j++)
    {
      temp_transform.transform.translation.x += g_transforms.at(j).at(i/pow(2, j)).transform.translation.x;
      temp_transform.transform.translation.y += g_transforms.at(j).at(i/pow(2, j)).transform.translation.y;
      temp_transform.transform.translation.z += g_transforms.at(j).at(i/pow(2, j)).transform.translation.z;
      temp_transform.transform.rotation = multiplyQuaternion(temp_transform.transform.rotation ,g_transforms.at(j).at(i/pow(2, j)).transform.rotation);
    }
    
    // Publish it
    mrgs_complete_map::LatestMapTF temp_latest;
    temp_latest.id = i;
    temp_latest.transform = temp_transform;
    pub2.publish(temp_latest);
  }
  
  /// Publish our new, shiny, complete map and report performance
  // Use a latched topic, so that the last map is accessible to new subscribers.
  ROS_DEBUG("Publishing new complete map.");
  g_aligned_maps.at(g_aligned_maps.size()-1).at(0).header.frame_id = "complete_map";
  pub1.publish(g_aligned_maps.at(g_aligned_maps.size()-1).at(0));
  ROS_INFO("Map processing took %fs.", (ros::Time::now() - init).toSec());
}

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "complete_map_node");
  ros::NodeHandle n;
  g_client = n.serviceClient<mrgs_alignment::align>("align");
  mrgs_alignment::align srv;
  ros::Subscriber sub2 = n.subscribe("foreign_maps", 1, processForeignMaps);
  pub1 = n.advertise<nav_msgs::OccupancyGrid>("complete_map", 10);
  pub2 = n.advertise<mrgs_complete_map::LatestMapTF>("remote_nav/remote_tf", 10);
  
  // ROS loop
  ros::spin();

  // Never to be called
  return 0;
}
