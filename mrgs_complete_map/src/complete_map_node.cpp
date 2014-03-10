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
#include <cstdlib>

// Global variables
// To be edited only by the processMap callback (we'll ignore this one for now and receive the local map via foreign map
// vector).
nav_msgs::OccupancyGrid::ConstPtr g_latest_local_map;
// To be edited only by the /foreign_maps callback
mrgs_data_interface::ForeignMapVector::ConstPtr g_latest_foreign_maps;
// To be edited only by the processForeignMaps callback
// Keeps the "dirtiness" (the need to be rebuilt) of aligned maps.
std::vector<std::vector<bool>> g_is_dirty;

void processMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  g_latest_local_map = map;
}

void processForeignMaps(const mrgs_data_interface::ForeignMapVector::ConstPtr& maps)
{
  /// Determine which maps have to be rebuilt
  // Compare sizes between the dirtiness matrix and the vector of maps we've received
  if(g_is_dirty.at(0).size() < maps->map_vector.size())
  {
    // The set of maps we've received is bigger than the one we had.
    // This is to be expected in the beginning of the mission, not so much towards the end.
    // Increment the dirtiness matrix, marking all new cells as dirty.
    while(g_is_dirty.at(0).size() < maps->map_vector.size())
    {
      // Increment one by one until we're at the right size?
    }
  }
  else if(g_is_dirty.at(0).size() > maps->map_vector.size())
  {
    // Something is quite wrong, we should probably report it.
  }
  
  // Check if the received maps have updates and mark as dirty accordingly
  // Make the received maps our current maps
  g_latest_foreign_maps = maps;
  
  /// Rebuild maps
  // Iterate through the dirtiness matrix, starting in row 1 (not 0), and rebuild 
  // all maps which depend on a "dirty" map. Newly-built maps are also marked as "dirty".
  // This process is repeated until we only have a single map to build, which will be our final map.
  
  /// Publish our new, shiny, complete map
  // Use a latched topic, so that the last map is accessible to new subscribers.
}

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "complete_map_node");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mrgs_alignment::align>("align");
  mrgs_alignment::align srv;
  //ros::Subscriber sub1 = n.subscribe("map", 1, processMap);
  ros::Subscriber sub2 = n.subscribe("foreign_maps", 1, processForeignMaps);
  ros::Publisher pub1 = n.advertise<nav_msgs::OccupancyGrid>("complete_map", 10);
  
  //ros::Rate r(1/30.0);
  
  // ROS loop
  ros::spin();
  /*while(ros::ok())
  {
    // Get all maps
    ros::spinOnce();
    /*
    if (client.call(srv))
    {
        
    }
    else
    {
      ROS_ERROR("Service call failed (probably no occupied cells in one of the supplied grids).");
      return 1;
    }*/
    r.sleep();
  }*/

  
  return 0;
}
