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
 * 
 * Methodology:
 *    1. Calculate how large the mapmerge grids must be in order to accomodate the incoming OccupancyGrids, and also for 
 * those grids to be able to rotate freely without loss of information.
 *    2. Copy incoming grids into mapmerge grids, taking into account the padding we've added, which implies a
 * translation step so that the cell that was previously the grid's center remains its center (which guarantees we won't
 * lose any information in rotation since rotations are done around the grid's center). We must also check for the
 * existence of at least one occupied cell in each map, or we'll get a segfault.
 *    3. Feed the copied grids into the algorithm.
 *    4. Apply the suggested rotation.
 *    5. (TODO) Determine if the to-be-translation will cause loss of data, and if so, accordingly pad the receiving map
 * in the necessary direction.
 *    6. Apply the suggested translation.
 *    7. Pack results into response message, including transformations (which implies calculating them).
 */

// ROS includes
#include "ros/ros.h"
#include "mrgs_alignment/align.h"
#include "ros/console.h"

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
#define MRGS_LOW_PROB_THRESH 10.0
#define MRGS_HIGH_PROB_THRESH 90.0

bool align(mrgs_alignment::align::Request &req, mrgs_alignment::align:: Response &res)
{
  ros::Time init = ros::Time::now();
  ROS_INFO("Received an alignment request. Alignment initiated.");
  ROS_DEBUG("Dimensions (h x w): %dx%d and %dx%d.", req.map1.info.height, req.map1.info.width,
                                                   req.map2.info.height, req.map2.info.width);
                                                   
  /// First step: padding and copying into mapmerge datatypes
  // Padding prevents us from losing parts of the map when rotating and translating the maps.
  // We add padding by adding to the final dimension of the map before copying them into mapmerge datatypes.
  // Determine if we need to add any padding
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
  }
  
  // Now we add rotational padding (so that we don't lose info in map rotation)
  int maximum_distance = ceil(sqrt((pow(map_final_r/2.0,2))+(pow(map_final_c/2.0,2))));
  int padding_rows = maximum_distance - ceil(map_final_r/2.0);
  int padding_cols = maximum_distance - ceil(map_final_c/2.0);
  if(padding_rows < 0) padding_rows = 0;
  if(padding_cols < 0) padding_cols = 0;
  map_final_r += 2*padding_rows;
  map_final_c += 2*padding_cols;
  
  // Transfer grid info into datatypes mapmerge can interpret
  // We'll assume values are either -1 for unknown, 100 for occupied and 0 for free.
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
  mapmerge::translate_map(a, temp_a, -padding_rows, -padding_cols);
  mapmerge::translate_map(b, temp_b, -padding_rows, -padding_cols);

  /// Second step: calculate and apply transformation
  // Get results (we only want one hypothesis, but we have to calculate several, or the results become invalid)
  int n_hypothesis = 4;
  ROS_DEBUG("Calculating hypotheses.");
  std::vector<mapmerge::transformation> hyp = mapmerge::get_hypothesis(a,b,n_hypothesis,1,false);
  //ROS_DEBUG("Best result: %f %d %d %d", hyp[0].ai, hyp[0].deltax, hyp[0].deltay, hyp[0].rotation);
  
  // Rototranslate map (missing: determine if translation will lose info and act accordingly)
  ROS_DEBUG("Applying transformation and merging.");
  mapmerge::grid_map c, d;
  float rotx, roty;
  mapmerge::rotate_map(c, b, hyp[0].rotation, 127, rotx, roty);
  mapmerge::translate_map(d, c, hyp[0].deltax, hyp[0].deltay);
  
  /// Third step: merge maps and pack into response
  // Merge map
  // d contains the aligned map, c can contain the merged map.
  // We assume a and d have equal dimensions (a and b should have been padded to avoid losses in rotation)
  // At the same time, we'll determine the region of interest, in case we want to crop.
  unsigned int roi_top_row = 0, roi_top_col = 0, roi_bottom_row = 0, roi_bottom_col = 0;
  bool in_roi = false, known_cell = false;
  for(int i = 0; i < map_final_r; i++)
  {
    for(int j = 0; j < map_final_c; j++)
    {
      c.grid.at(i).at(j) = 127;
      if(a.grid.at(i).at(j) == 255 || d.grid.at(i).at(j) == 255)
      {
        c.grid.at(i).at(j) = 255;
        known_cell = true;
      }
      if(a.grid.at(i).at(j) == 0 || d.grid.at(i).at(j) == 0) 
      {
        c.grid.at(i).at(j) = 0;
        known_cell = true;
      }
      if(known_cell == true)
      {
        known_cell = false;
        if(in_roi == false)
        {
          in_roi = true;
          roi_top_row = i;
          roi_top_col = j;
        }
        else
        {
          roi_bottom_row = i;
          if(j < roi_top_col)
            roi_top_col = j;
          if(j > roi_bottom_col)
            roi_bottom_col = j;
        }
      }
    }
  }
  
  // DEBUG: Write maps to disk for viewing
  //mapmerge::save_map_to_file(a, "/home/vsantos/lol/in1.png");
  //mapmerge::save_map_to_file(b, "/home/vsantos/lol/in2.png");
  //mapmerge::save_map_to_file(c, "/home/vsantos/lol/out.png");
  
  // Pack results into response message
  ROS_DEBUG("Packing results into response.");
  if(req.crop == false)
  {
    res.merged_map.data.resize(map_final_r*map_final_c);
    res.merged_map.info.resolution = req.map1.info.resolution;
    res.merged_map.header.frame_id = "map";
    res.merged_map.info.width = map_final_c;
    res.merged_map.info.height = map_final_r;
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
  }
  else
  {
    map_final_r = roi_bottom_row-roi_top_row+1;
    map_final_c = roi_bottom_col - roi_top_col+1;
    res.merged_map.data.resize(map_final_r*map_final_c);
    res.merged_map.info.resolution = req.map1.info.resolution;
    res.merged_map.header.frame_id = "map";
    res.merged_map.info.width = map_final_c;
    res.merged_map.info.height = map_final_r;
    res.merged_map.info.map_load_time = ros::Time::now();
    k = 0;
    for(int i = 0; i < c.get_rows(); i++)
    {
      for(int j = 0; j < c.get_cols();j++)
      {
        if(i >= roi_top_row && j >= roi_top_col && i <= roi_bottom_row && j <= roi_bottom_col)
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
    }
  }
  res.success_coefficient = hyp[0].ai;
  
  /// Final step: calculate and pack transforms into response.
  ROS_DEBUG("Calculating and packing transforms.");
  // From map1 to merged_map
  res.transform1.transform.rotation.x = 0;
  res.transform1.transform.rotation.y = 0;
  res.transform1.transform.rotation.z = 0;
  res.transform1.transform.rotation.w = 1;
  res.transform1.transform.translation.x = padding_cols * res.merged_map.info.resolution;
  res.transform1.transform.translation.y = padding_rows * res.merged_map.info.resolution;
  res.transform1.transform.translation.z = 0;
  
  // From map2 to merged_map
  // Rotation
  float deg_to_rad = 0.01745329251; // = pi/180, precalculated for performance.
  float theta = deg_to_rad * -1 * hyp[0].rotation;
  float sin_theta = sin(theta/2);
  float cos_theta = cos(theta/2);
  float quaternion_magnitude = sqrtf(pow(sin_theta, 2) + pow(cos_theta,2));
  res.transform2.transform.rotation.x = 0;
  res.transform2.transform.rotation.y = 0;
  res.transform2.transform.rotation.z = cos_theta/quaternion_magnitude;
  res.transform2.transform.rotation.w = sin_theta/quaternion_magnitude;
  // Padding translation
  res.transform2.transform.translation.x = padding_cols * res.merged_map.info.resolution;
  res.transform2.transform.translation.y = padding_rows * res.merged_map.info.resolution;
  res.transform2.transform.translation.z = 0; 
  // Rotational translation (these equations should be explained somewhere)
  // Basically, since we are rotating from the center, we introduce an extra translation on the corner that is 
  // calculated through these equations.
  // I'll reuse some variables, but their meanings will be different.
  float half_h = (map_final_r/2.0) * res.merged_map.info.resolution;
  float half_w = (map_final_c/2.0) * res.merged_map.info.resolution;
  theta = M_PI - atan(half_h/half_w);
  float dist = sqrt(pow(half_h, 2) + pow(half_w, 2));
  res.transform2.transform.translation.x += (dist*cosf(theta - (hyp[0].rotation*deg_to_rad))) - (dist*cosf(theta));
  res.transform2.transform.translation.y += (dist*sinf(theta - (hyp[0].rotation*deg_to_rad))) - (dist*sinf(theta));
  // Merging translation
  res.transform2.transform.translation.x += hyp[0].deltax;
  res.transform2.transform.translation.y += hyp[0].deltay;
  
  
  // Final report
  ROS_INFO("Results sent. Total service time was %fs.", (ros::Time::now()-init).toSec());
  
  // Successfully return
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "align_node");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("align", align);
  ros::spin();

  return 0;
}
