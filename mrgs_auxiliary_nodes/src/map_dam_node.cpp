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
 * The node intercepts the /map topic and publishes a new topic, which the data interface subscribes to. Since the data
 * interface node is the entry point for maps in the system, this is enough to control the way maps enter the system.
 * This node receives raw maps from SLAM, and crops them, removing any excess padding, in order to facilitate the 
 * trasmission and alignment of the map.
 */
// ROS includes
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdlib>

// LocalMap message:
#include "mrgs_data_interface/LocalMap.h"

class MapDam{
  public:
  // Callback for /map
  void processUnfilteredMap(const nav_msgs::OccupancyGrid::ConstPtr& unfiltered_map)
  {
    // Allocate a new publish-able map:
    mrgs_data_interface::LocalMap filtered_map;
    // Determine map's region of interest, for cropping
    int top_line = -1, bottom_line = -1, left_column = -1, right_column = -1;
    for(int i = 0; i < unfiltered_map->data.size(); i++)
    {
      if(unfiltered_map->data.at(i) != -1)
      {
        int line = i/unfiltered_map->info.width;
        int col = i%unfiltered_map->info.width;
        if(top_line == -1)
          top_line = line;
        if(left_column == -1)
          left_column = col;
        if(col < left_column)
          left_column = col;
        if(col > right_column)
          right_column = col;
        if(line > bottom_line)
          bottom_line = line;
      }
    }
    ROS_INFO("Smallest rectangle found: (%d,%d) to (%d,%d). Original: (%dx%d).", top_line, left_column, bottom_line, right_column, unfiltered_map->info.height, unfiltered_map->info.width);
    // Copy map into message and publish
    filtered_map.filtered_map = *unfiltered_map;
    if(listener->canTransform ("/base_link", "/map", ros::Time(0)))
    {
      tf::StampedTransform map_to_base_link;
      listener->lookupTransform(std::string("/map"), std::string("/base_link"), ros::Time(0), map_to_base_link);
      tf::transformStampedTFToMsg(map_to_base_link, filtered_map.map_to_base_link);
    }
    else
    {
    }
    last_map = filtered_map;
    //map_publisher.publish(filtered_map);
  }
  // Constructor
  MapDam(ros::NodeHandle* n_p)
  {
    first_map = true;
    map_publisher = n_p->advertise<mrgs_data_interface::LocalMap>("mrgs/local_map", 2);
    map_subscriber = n_p->subscribe("map", 2, &MapDam::processUnfilteredMap, this);
    listener = new tf::TransformListener;
  }
  
  ~MapDam()
  {
    delete listener;
  }
  
  private:
  // Latest received map
  mrgs_data_interface::LocalMap last_map;
  // Is this the first map we've ever received?
  bool first_map;
  // Map publisher
  ros::Publisher map_publisher;
  // Map subscriber
  ros::Subscriber map_subscriber;
  // TF listener
  tf::TransformListener *listener;
};


int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "remote_map_node");
  ros::NodeHandle n;
  MapDam dam(&n);
  
  // ROS loop
  ros::spin();

  // Never to be called
  return 0;
}
