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
 * Currently, we simply insert all the maps we receive into the system, acting as an over-complicated topic redirector.
 * However, the plans are to integrate intelligence into this node.
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
    mrgs_data_interface::LocalMap filtered_map;
    // Save the received map as the latest received map
    last_map = *unfiltered_map;
    // Determine map's region of interest, for cropping
    int top_line, bottom_line, left_column, right_column;
    for(int i = 0; i < last_map.data.size(); i++)
    {
    }
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
    map_publisher.publish(filtered_map);
  }
  // Constructor
  MapDam(ros::NodeHandle* n_p)
  {
    first_map = true;
    map_publisher = n_p->advertise<mrgs_data_interface::LocalMap>("mrgs/local_map", 2);
    map_subscriber = n_p->subscribe("map", 2, &MapDam::processUnfilteredMap, this);
    listener = new tf::TransformListener;
  }
  
  private:
  // Latest received map
  nav_msgs::OccupancyGrid last_map;
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
