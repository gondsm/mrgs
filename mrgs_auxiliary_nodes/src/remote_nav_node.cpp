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
 * remote_nav_node
 * 
 * Summary:
 * This node ensures that TFs and poses are published into the internal network at the correct times.
 * 
 * Methodology:
 * This node is responsible for publishing the following data:
 * -> complete_map to local map (from each robot) TFs;
 * -> local map (from each robot) to local odom TFs;
 * -> remote poses.
 * This information is published repeatedly at (desireably) 10Hz, even if no new info is received, and is meant to
 * facilitate the visualization of the mission's progress. The relatively high rate of transmission is meant
 * to keep the TF tree alive with the latest information gathered in order to ensure the robot is always capable of
 * transforming between its local complete map frame and the frames of every local map, as well as between those maps
 * and the associated poses.
 */
// ROS includes
#include "ros/ros.h"
#include "mrgs_complete_map/LatestMapTF.h"
#include "mrgs_data_interface/LatestRobotPose.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include <cstdlib>

// Global variables
// Holds all current complete_map to map transforms
std::vector<tf::Transform*> g_map_transform_vector;
// Holds all current map to base_link transforms
std::vector<tf::Transform*> g_base_transform_vector;

void processTF(const mrgs_complete_map::LatestMapTF::ConstPtr& remote_transform)
{
  // Add the received transform to the vector.
  ROS_INFO("Processing new complete_map to map transform.");  
  while(g_map_transform_vector.size() < remote_transform->id+1)
    g_map_transform_vector.push_back(NULL);
  
  // Copy received transform to temporary variable
  tf::StampedTransform new_transform;
  tf::transformStampedMsgToTF(remote_transform->transform, new_transform);
  
  // Copy from temp to vector
  g_map_transform_vector.at(remote_transform->id) = new tf::Transform(new_transform);
  
}

void processPose(const mrgs_data_interface::LatestRobotPose::ConstPtr& remote_pose)
{
  // Add the received transform to the vector.
  ROS_INFO("Processing new map to base_link transform.");  
  while(g_base_transform_vector.size() < remote_pose->id+1)
    g_base_transform_vector.push_back(NULL);
  
  // Copy received transform to temporary variable
  tf::StampedTransform new_transform;
  tf::transformStampedMsgToTF(remote_pose->transform, new_transform);
  
  // Copy from temp to vector
  g_base_transform_vector.at(remote_pose->id) = new tf::Transform(new_transform);
}



int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "remote_nav_node");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("mrgs/remote_poses", 10, processPose);
  ros::Subscriber sub2 = n.subscribe("mrgs/remote_tf", 10, processTF);
  tf::TransformBroadcaster broadcaster;
  
  // ROS loop
  //ros::spin();
  ros::Rate r(10);
  while(ros::ok())
  {
    // Broadcast all current data:
    //    Iterate through the various vectors, publishing data in the correct
    //    TF frames. Don't forget to correctly build the header for the stamped
    //    StampedPoses.
    char frame[30];
    char child_frame[30];
    for(int i = 0; i < g_map_transform_vector.size(); i++)
    {
      if(g_map_transform_vector.at(i) != NULL)
      {
        sprintf(frame, "/complete_map");
        sprintf(child_frame, "/robot_%d/map", i);
        broadcaster.sendTransform(tf::StampedTransform(*g_map_transform_vector.at(i), ros::Time::now(), frame, child_frame));
      }
    }
    
    for(int i = 0; i < g_base_transform_vector.size(); i++)
    {
      sprintf(frame, "/robot_%d/map");
      sprintf(frame, "/robot_%d/base_link", i);
      if(g_base_transform_vector.at(i) != NULL)
      {
        broadcaster.sendTransform(tf::StampedTransform(*g_base_transform_vector.at(i), ros::Time::now(), frame, child_frame));
      }
    }
    
    ros::spinOnce();
    r.sleep();
  }

  // Never to be called
  return 0;
}
