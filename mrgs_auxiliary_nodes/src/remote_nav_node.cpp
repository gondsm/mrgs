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
#include "mrgs_auxiliary_nodes/LatestMapTF.h"
#include "mrgs_auxiliary_nodes/LatestRobotPose.h"
#include <cstdlib>

// Global variables


void processPose(const mrgs_auxiliary_nodes::LatestMapTF::ConstPtr& remote_pose)
{
}

void processTF(const mrgs_auxiliary_nodes::LatestRobotPose::ConstPtr& remote_transform)
{
}



int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "remote_nav_node");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("remote_nav/remote_poses", 10, processPose);
  ros::Subscriber sub2 = n.subscribe("remote_nav/remote_tf", 10, processTF);
  
  // ROS loop
  ros::spin();

  // Never to be called
  return 0;
}
