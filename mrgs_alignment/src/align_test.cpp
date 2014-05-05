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
 * align_test:
 * Small program to test the operation of the align_node.
 * Example call: 
 * rosrun mrgs_alignment align_test `rospack find mrgs_alignment`/datasets/intel.txt `rospack find mrgs_alignment`/datasets/intel90.txt 800 800 
 */
 
// ROS includes
#include "ros/ros.h"
#include "mrgs_alignment/align.h"
#include <cstdlib>
#include <ros/console.h>

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

// My includes
#include <typeinfo>  // To determine some datatypes

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "align_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mrgs_alignment::align>("align");
  mrgs_alignment::align srv;

  // Argument loading
  if(argc < 5){
    ROS_INFO("Usage: image1.txt image2.txt rows cols");
    return -1;
  }
  ROS_INFO("Loading %s and %s.", argv[1], argv[2]);
  mapmerge::grid_map a,b;
  a.load_map(atoi(argv[3]),atoi(argv[4]),argv[1]);
  b.load_map(atoi(argv[3]),atoi(argv[4]),argv[2]);
  
  // Map conversion to OccupancyGrid, for service requesting
  // a.grid is a vector of unsigned int vectors, which is, at the very
  // least, an interesting choice.
  srv.request.map1.data.resize(a.get_rows()*a.get_cols());
  srv.request.map2.data.resize(b.get_rows()*b.get_cols());

  // Copy data into vectors (assuming both are the same size, no reason not to)
  int k = 0;
  for(int i = 0; i < a.get_rows(); i++)
  {
    for(int j = 0; j < a.get_cols();j++)
    {
      switch(a.grid.at(i).at(j))
      {
      case 0:
        srv.request.map1.data.at(k) = 100;
        break;
      case 127:
        srv.request.map1.data.at(k) = -1;
        break;
      case 255:
        srv.request.map1.data.at(k) = 0;
        break;
      default:
        ROS_INFO("Found strange cell in map a at (%d,%d).", i, j);
      }
      switch(b.grid.at(i).at(j))
      {
      case 0:
        srv.request.map2.data.at(k) = 100;
        break;
      case 127:
        srv.request.map2.data.at(k) = -1;
        break;
      case 255:
        srv.request.map2.data.at(k) = 0;
        break;
      default:
        ROS_INFO("Found strange cell in map b at (%d,%d).", i, j);
      }
      k++;
    }
  }
  srv.request.map1.info.width = srv.request.map2.info.width = a.get_cols();
  srv.request.map1.info.height = srv.request.map2.info.height = a.get_rows();

  // Local test for result comparison
  int n_hypothesis = 4;
  std::vector<mapmerge::transformation> hyp = mapmerge::get_hypothesis(a,b,n_hypothesis,1,false);
  ROS_INFO("Local Results:");
  for ( unsigned int i = 0 ; i < n_hypothesis ; i++ )
    ROS_INFO("Hypothesis %d: %f %d %d %d", i+1,hyp[i].ai, hyp[i].deltax, hyp[i].deltay, hyp[i].rotation);

  // Service call
  if (client.call(srv))
  {
    ROS_INFO("Received results: %f", srv.response.success_coefficient);
  }
  else
  {
    ROS_ERROR("Service call failed.");
    //return 1;
  }

  return 0;
}
