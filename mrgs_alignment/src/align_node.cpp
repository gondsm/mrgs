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
 * align_node
 *
 * Summary:
 * This node receives, via ROS service, a pair of occupancy grids, and returns a single, aligned and merged, grid.
 * This node applies the algorithm developed by Stefano Carpin and presented on his papers from 2007 and 2008 (cited on
 * my thesis).
 * I've been careful to ensure that this node communicates via standard ROS datatypes, so that it is easily swapped by
 * another available map fusion technique.
 * It is important to note that the node is currently ignoring requests for cropped grids.
 *
 * Methodology:
 *    1. Calculate how large the mapmerge grids must be in order to accomodate the incoming OccupancyGrids.
 *    2. Copy incoming grids into mapmerge grids, taking into account the padding we need to add added. We must also
 * check for the existence of at least one occupied cell in each map, or we'll get a runtime exception.
 *    3. Feed the copied grids into the algorithm.
 *    4. Apply the suggested rotation.
 *    5. Apply the suggested translation.
 *    6. Calculate the transforms.
 *    7. Pack results into response message, including transformations (which implies calculating them).
 */

// ROS includes
#include "ros/ros.h"
#include "mrgs_alignment/align.h"
#include "ros/console.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

// mapmerge includes
#include "mapmerge/grid_map.h"
#include "mapmerge/io.h"
#include "mapmerge/common.h"
#include "mapmerge/hough.h"
#include "mapmerge/manipulatemap.h"
#include <iterator>
#include <iostream>
#include <algorithm>
#include <stdlib.h>

// Other includes
//#define _USE_MATH_DEFINES
#include <math.h>

// Thresholds for deciding whether cells are occupied or free
#define MRGS_LOW_PROB_THRESH 10
#define MRGS_HIGH_PROB_THRESH 70

// Number of hypothesis we calculate. The bigger this number, the better our chance to find the right transformation,
// and the more CPU we need.
int g_n_hypothesis;
int n = 0;

// DEBUG: Pose publisher
ros::Publisher pose_publisher;

bool align(mrgs_alignment::align::Request &req, mrgs_alignment::align:: Response &res)
{
  /// Start counting time
  // This is so we can report how long it took to complete the alignment, and adjust the number of hypotheses.
  ros::Time init = ros::Time::now();
  ROS_INFO("Received an alignment request. Alignment initiated.");
  ROS_DEBUG("Dimensions (h x w): %dx%d and %dx%d.", req.map1.info.height, req.map1.info.width,
                                                   req.map2.info.height, req.map2.info.width);

  /// Calculate dimensioning padding
  // This ensures that both maps get input into mapmerge with the same dimensions
  int map_final_r = 0, map_final_c = 0;
  if(req.map1.info.height == req.map2.info.height && req.map1.info.width == req.map2.info.width)
  {
    ROS_DEBUG("Maps have equal dimensions, only need to apply rotational padding.");
    map_final_r = req.map1.info.height;
    map_final_c = req.map1.info.width;
  }
  else
  {
    ROS_DEBUG("Map dimensions are different. Padding is needed.");
    // Determine which operations to apply
    if(req.map1.info.height != req.map2.info.height)
    {
      // Different heights, need to pad
      if(req.map1.info.height > req.map2.info.height)
      {
        // Add height to map2 until its height is the same as map1
        map_final_r = req.map1.info.height;
      }
      else
      {
        // Add height to map1 until its height is the same as map2
        map_final_r = req.map2.info.height;
      }
    }
    else
    {
      map_final_r = req.map1.info.height;
    }
    if(req.map1.info.width != req.map2.info.width)
    {
      // Different widths, need to pad
      if(req.map1.info.width > req.map2.info.width)
      {
        // Add width to map2 until its width is the same as map1
        map_final_c = req.map1.info.width;
      }
      else
      {
        // Add width to map1 until its width is the same as map2
        map_final_c = req.map2.info.width;
      }
    }
    else
    {
      map_final_c = req.map1.info.width;
    }
  }

  // Now we add rotational padding
  // Padding prevents us from losing parts of the map when rotating and translating the maps.
  // We add padding by adding to the final dimension of the map before copying them into mapmerge datatypes.
  // This step is necessary since we are assuming all the maps we receive have been cropped by the map dam node.
  int maximum_distance = ceil(sqrt((pow(map_final_r/2.0,2))+(pow(map_final_c/2.0,2))));
  int padding_rows = maximum_distance - ceil(map_final_r/2.0);
  int padding_cols = maximum_distance - ceil(map_final_c/2.0);
  if(padding_rows < 0) padding_rows = 0;
  if(padding_cols < 0) padding_cols = 0;
  map_final_r += 2*padding_rows;
  map_final_c += 2*padding_cols;
  //int padding_rows = 0;
  //int padding_cols = 0;

  /// Copy maps into mapmerge datatypes
  // Transfer grid info into datatypes mapmerge can interpret
  // We'll assume values are either -1 for unknown, > MRGS_HIGH_PROB_THRESH for occupied, and < MRGS_LOW_PROB_THRES for
  // free. Check beginning of code.
  // Different values will be classified as unknown.
  ROS_DEBUG("Final dimensions: rows=%d, cols=%d. Copying into mapmerge data.", map_final_r, map_final_c);
  mapmerge::grid_map temp_a(map_final_r, map_final_c),temp_b(map_final_r, map_final_c);
  bool exists_occupied1 = false, exists_occupied2 = false;
  int k = 0, k1 = 0, k2 = 0;  // Linear counters
  for(int i = 0; i < map_final_r; i++)
  {
    for(int j = 0; j < map_final_c;j++)
    {
      // Determine which value to attribute to the current cell
      // (and attribute it)
      if(i < req.map1.info.height && j < req.map1.info.width)
      {
        // If (i,j) are inside the original dimensions
        if(req.map1.data.at(k1) == -1)
          temp_a.grid.at(i).at(j) = 127;
        else if(req.map1.data.at(k1) < MRGS_LOW_PROB_THRESH)
          temp_a.grid.at(i).at(j) = 255;
        else if (req.map1.data.at(k1) > MRGS_HIGH_PROB_THRESH)
          {
            temp_a.grid.at(i).at(j) = 0;
            if(!exists_occupied1) exists_occupied1 = true;
          }
          else
            temp_a.grid.at(i).at(j) = 127;
        // Increment linear counter
        k1++;
      }
      else
      {
        // Insert an unknown cell
        temp_a.grid.at(i).at(j) = 127;
      }

      if(i < req.map2.info.height && j < req.map2.info.width)
      {
        // If (i,j) are inside the original dimensions
        if(req.map2.data.at(k2) == -1)
          temp_b.grid.at(i).at(j) = 127;
        else if(req.map2.data.at(k2) < MRGS_LOW_PROB_THRESH)
          temp_b.grid.at(i).at(j) = 255;
        else if (req.map2.data.at(k2) > MRGS_HIGH_PROB_THRESH)
          {
            temp_b.grid.at(i).at(j) = 0;
            if(!exists_occupied2) exists_occupied2 = true;
          }
          else
            temp_b.grid.at(i).at(j) = 127;

        // Increment linear counter
        k2++;
      }
      else
      {
        // Insert an unknown cell
        temp_b.grid.at(i).at(j) = 127;
      }

    }
  }

  // Break if there are no occupied cells
  if(!exists_occupied1 || !exists_occupied2){
    ROS_ERROR("At least one of the provided grids contain no occupied cells. Terminating.");
    res.success_coefficient = -1;
    return true;
  }

  // Translate grids to re-center:
  mapmerge::grid_map a,b;
  mapmerge::translate_map(a, temp_a, -padding_cols, -padding_rows);
  mapmerge::translate_map(b, temp_b, -padding_cols, -padding_rows);

  /// Call mapmerge to determine the transformation
  ROS_DEBUG("Calculating hypotheses.");
  std::vector<mapmerge::transformation> hyp = mapmerge::get_hypothesis(a,b,g_n_hypothesis,1,false);


  /// DEBUG
  //hyp[0].rotation = 0;
  //hyp[0].deltax = 0;
  //hyp[0].deltay = 0;

  // Show all hypotheses found
  for(int i = 0; i < g_n_hypothesis; i++)
    ROS_DEBUG("Hypothesis %d: ai=%f x=%d y=%d theta=%d", i, hyp[i].ai, hyp[i].deltax, hyp[i].deltay, hyp[i].rotation);

  /// Merge maps and pack into response
  // c will contain the rotated map, d will contain the roto-translated map
  ROS_DEBUG("Applying transformation and merging.");
  mapmerge::grid_map c, d;
  float rotx, roty;
  mapmerge::rotate_map(c, b, hyp[0].rotation, 127, rotx, roty);
  // DEBUG: Write first maps to disk
  /*char buffer[50];
  sprintf(buffer,"../results/%da.png",n);
  mapmerge::save_map_to_file(a, buffer);
  sprintf(buffer,"../results/%dtemp_a.png",n);
  mapmerge::save_map_to_file(temp_a, buffer);
  sprintf(buffer,"../results/b%d.png",n);
  mapmerge::save_map_to_file(b,buffer);
  sprintf(buffer,"../results/c%d.png",n);
  mapmerge::save_map_to_file(c,buffer);*/
  mapmerge::translate_map(d, c, hyp[0].deltax, hyp[0].deltay);
  // DEBUG: Write translated map to disk
  /*sprintf(buffer,"../results/d%d.png",n);
  mapmerge::save_map_to_file(d,buffer);*/

  // Merge maps
  for(int i = 0; i < map_final_r; i++)
  {
    for(int j = 0; j < map_final_c; j++)
    {
      c.grid.at(i).at(j) = 127;
      if(a.grid.at(i).at(j) == 255 || d.grid.at(i).at(j) == 255)
      {
        c.grid.at(i).at(j) = 255;
      }
      if(a.grid.at(i).at(j) == 0 || d.grid.at(i).at(j) == 0)
      {
        c.grid.at(i).at(j) = 0;
      }
    }
  }

  // DEBUG: Write final map to disk
  /*sprintf(buffer,"../results/final%d.png",n);
  mapmerge::save_map_to_file(c, buffer);*/

  // Pack results into response message
  ROS_DEBUG("Packing results into response.");
  res.merged_map.data.resize(map_final_r*map_final_c);
  res.merged_map.info.resolution = req.map1.info.resolution;
  res.merged_map.header.frame_id = "/complete_map";
  res.merged_map.info.width = map_final_c;
  res.merged_map.info.height = map_final_r;
  res.merged_map.info.origin = req.map1.info.origin;
  res.merged_map.info.origin.position.x -= (padding_cols)*req.map1.info.resolution;
  res.merged_map.info.origin.position.y -= (padding_rows)*req.map1.info.resolution;

  k = 0;
  for(int i = 0; i < c.get_rows(); i++)
  {
    for(int j = 0; j < c.get_cols();j++)
    {
      switch(c.grid.at(i).at(j))
      {
      case 0:
        res.merged_map.data.at(k) = 100;
        break;
      case 127:
        res.merged_map.data.at(k) = -1;
        break;
      case 255:
        res.merged_map.data.at(k) = 0;
        break;
      default:
        res.merged_map.data.at(k) = -1;
        ROS_DEBUG("Found strange cell in map c at (%d,%d). Defaulting to unknown.", i, j);
      }
      k++;
    }
  }
  res.success_coefficient = hyp[0].ai;

  /// Compensate origin translation due to padding
  req.map1.info.origin.position.x -= (padding_cols)*req.map1.info.resolution;
  req.map1.info.origin.position.y -= (padding_rows)*req.map1.info.resolution;
  req.map2.info.origin.position.x -= (padding_cols)*req.map1.info.resolution;
  req.map2.info.origin.position.y -= (padding_rows)*req.map1.info.resolution;

  /// Calculate transformations
  ROS_DEBUG("Calculating and packing transforms.");
  // From map1 to merged_map
  res.transform1.transform.rotation.x = 0;
  res.transform1.transform.rotation.y = 0;
  res.transform1.transform.rotation.z = 0;
  res.transform1.transform.rotation.w = 1;
  //res.transform1.transform.translation.x = padding_cols * res.merged_map.info.resolution;
  //res.transform1.transform.translation.y = padding_rows * res.merged_map.info.resolution;
  res.transform1.transform.translation.x = 0;
  res.transform1.transform.translation.y = 0;
  res.transform1.transform.translation.z = 0;

  // From map2 to merged_map
  // Rotation
  float deg_to_rad = 0.01745329251; // = pi/180, precalculated for performance.
  float theta = deg_to_rad *  hyp[0].rotation;

  // Center to center
  tf::Transform center_to_center;
  tf::Quaternion rotation;

  // Rotation
  center_to_center.setIdentity();
  rotation.setRPY(0, 0, theta);
  rotation.normalize();
  center_to_center.setRotation(rotation);
  // Translation
  //center_to_center.setOrigin(tf::Vector3(-hyp[0].deltay * res.merged_map.info.resolution, hyp[0].deltax* res.merged_map.info.resolution, 0));
  center_to_center.setOrigin(tf::Vector3(-hyp[0].deltax * res.merged_map.info.resolution, -hyp[0].deltay* res.merged_map.info.resolution, 0));

  // Map1 to origin
  tf::Transform map1_to_origin, map2_to_origin, origin_to_center;
  tf::quaternionMsgToTF(req.map1.info.origin.orientation, rotation);
  map1_to_origin.setRotation(rotation);
  map1_to_origin.setOrigin(tf::Vector3(req.map1.info.origin.position.x, req.map1.info.origin.position.y, req.map1.info.origin.position.z));
  // Map2 to origin
  tf::quaternionMsgToTF(req.map2.info.origin.orientation, rotation);
  map2_to_origin.setRotation(rotation);
  map2_to_origin.setOrigin(tf::Vector3(req.map2.info.origin.position.x, req.map2.info.origin.position.y, req.map2.info.origin.position.z));
  // Origin to center
  origin_to_center.setIdentity();
  origin_to_center.setOrigin(tf::Vector3((res.merged_map.info.width*res.merged_map.info.resolution)/2.0, (res.merged_map.info.height*res.merged_map.info.resolution)/2.0, 0));

  // Map to map
  //tf::Transform map1_to_map2 = map1_to_origin*origin_to_center*center_to_center*origin_to_center.inverse()*map2_to_origin.inverse();
  tf::Transform map1_to_map2;
  map1_to_map2.setIdentity();
  map1_to_map2 *= map1_to_origin;
  map1_to_map2 *= origin_to_center;
  map1_to_map2 *= center_to_center;
  map1_to_map2 *= origin_to_center.inverse();
  map1_to_map2 *= map2_to_origin.inverse();

  // Pack into response
  tf::StampedTransform stamped(map1_to_map2, ros::Time::now(), "foo", "bar");
  tf::transformStampedTFToMsg(stamped, res.transform2);

  // DEBUG: Show origins
  geometry_msgs::PoseStamped debug_pose;
  debug_pose.pose = req.map2.info.origin;
  debug_pose.header.frame_id = std::string("/robot_1/map");
  //pose_publisher.publish(debug_pose);


  /// Adjust the number of hypotheses to calculate next according to this performance
  double total_time = (ros::Time::now()-init).toSec();
  ROS_INFO("Total service time was %fs.", total_time);
  n++;
  if(total_time > 5.0 && g_n_hypothesis > 8)
  {
    g_n_hypothesis/=2;
    ROS_INFO("Merging took longer than 5 seconds. Cutting the number of hypotheses to %d.", g_n_hypothesis);
  }
  else if(total_time > 2.0 && g_n_hypothesis > 6)
  {
    g_n_hypothesis/=1.5;
    ROS_INFO("Merging took longer than 3 seconds. Cutting the number of hypotheses to %d.", g_n_hypothesis);
  }
  else if(total_time < 2.0)
  {
    g_n_hypothesis++;
    ROS_INFO("Merging took less than 2 seconds. Incrementing the number of hypotheses to %d.", g_n_hypothesis);
  }

  /// Successfully return
  return true;
}

int main(int argc, char **argv)
{
  /// ROS Init
  ros::init(argc, argv, "align_node");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("align", align);

  // DEBUG: initialize pose publisher
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/map_1_origin", 10);

  /// Calibration
  mapmerge::grid_map a,b;
  if(a.load_map(800,800,"../calibration/intel.txt")==1 || b.load_map(800,800,"../calibration/intel90.txt")==1)
  {
    ROS_WARN("Calibration files could not be opened. Calibration will not be performed. First few executions may be slow.");
    g_n_hypothesis = 4;
  }
  else
  {
    ROS_INFO("Starting alignment node calibration.");
    ros::Time calibration_init = ros::Time::now();
    std::vector<mapmerge::transformation> hyp = mapmerge::get_hypothesis(a,b,1,1,false);
    ROS_INFO("Calibration hypothesis: ai=%f x=%d y=%d theta=%d", hyp[0].ai, hyp[0].deltax, hyp[0].deltay, hyp[0].rotation);
    ros::Duration calibration_time = (ros::Time::now()-calibration_init);
    g_n_hypothesis = ceil(10/calibration_time.toSec());
    if(g_n_hypothesis < 4)
      g_n_hypothesis = 4;
    ROS_INFO("Getting a hypothesis took %fs. Setting the number of hypotheses to %d.", calibration_time.toSec(), g_n_hypothesis);
    // Free memory
    a.resize_map(1,1);
    b.resize_map(1,1);
  }

  /// Spin
  ros::spin();

  return 0;
}
