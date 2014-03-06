 /* AUTORIGHTS
 Copyright (c) 2007 The Regents of the University of California.
 All Rights Reserved.
 
 Created by Stefano Carpin
 University of California, Merced, Robotics Lab - School of Engineering
 
 Permission to use, copy, modify, and distribute this software and its
 documentation for educational, research and non-profit purposes,
 without fee, and without a written agreement is hereby granted,
 provided that the above copyright notice, this paragraph and the
 following three paragraphs appear in all copies.
 
 This software program and documentation are copyrighted by The Regents
 of the University of California. The software program and
 documentation are supplied "as is", without any accompanying services
 from The Regents. The Regents does not warrant that the operation of
 the program will be uninterrupted or error-free. The end-user
 understands that the program was developed for research purposes and
 is advised not to rely exclusively on the program for any reason.
 
 IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
 INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND
 ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE UNIVERSITY OF
 CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS"
 BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATIONS TO PROVIDE
 MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

*/

/*! \file manipulatemap.h
  \brief Various functions to manipulate maps represented as occupancy grids.

  Actually in a clean (next version?) implementation many of these functions should
  be moved and become members of the grid_map class.
*/


#ifndef MANIPULATE_MAP
#define MANIPULATE_MAP

#include "grid_map.h"
#include <list>

namespace mapmerge {

  /*!
    Turns all grid cells whose status is unknown to free. Other cells are left the way they are. 
    @param out modified map
    @param in map to modify
   */
void cast_image_bw(grid_map& out,const grid_map& in);

/*!
  Creates a new map by translating an existing one. The new one have the same
size as the old one. Points "falling out" of the new map are ignored. Other cells
are initialized to the unknownn status
@param  out translated map
@param in initial map
@param dx translation along the x axis (columns)
@param dy translation along the y axis (rows)
 */
 void translate_map(grid_map& out,const grid_map& in,int dx,int dy);

 /*!
   Creates a new map rotating an existing one. Rotation is performed about the center
   of the map. The algorithm returns also the coordinates of the center of rotation.
   @param out rotate map
   @param in input map
   @param thetaDeg rotation angle in degrees
   @param filler value to use for tiles in the new map that are not initialized after rotation
   @param outtx column coordinate of the center
   @param outty row coordinate of the center
 */
void rotate_map(grid_map& out,const grid_map& in,int thetaDeg,unsigned char filler,float& outtx,float& outty);

/*! 
  Checks if the given grid_map is valid, i.e. if it contains only cells whose status is
  free, occupied or unknown. Useful only for debugging purposes (should probably be moved as
  a member method of grid_map).
  @param in map to check
  @return true if no anomaly is detected, false otherwise
 */
bool check_map(const grid_map& in);

/*!
  Restores a map to a valid status, i.e. containing only free, occupied and unknown cells.
  Useful for restoring a map that failed the check_map test. This should also be moved
  eventually as a member of grid_map.
  \param in map to be restored
 */
void restore_map(grid_map& in);

/*!
  Produces a new map by overlapping two existing ones. Important: the two maps
  must have the same size and must use the same encoding values for the cells
  @param out overlapped map
  @param m1 first map
  @param m2 second map
 */
void overlap_maps(grid_map& out,const grid_map& m1,const grid_map& m2);

/*!
  Saves the map to a given file in graphical formt. The filename extension determines 
  the file format. Since this is done via opencv, all and only file opencv supported file
  formats may be used
  @param to_save map to save
  @param fn file name to use
 */
void save_map_to_file(const grid_map& to_save,const char* fn);

/*!
  Produces a new map by roto-translating an existing one.
  @param out new map
  @param in map to be transformed
  @param thetaDeg rotation in degrees
  @param tx translation along x (columns)
  @param ty translation along y (row)
  @param filler character to use to fill cells that are not the image of any cell of the starting map
 */
void raw_transform_map(grid_map& out,const grid_map& in,float thetaDeg,float tx,float ty,unsigned char filler);

/*!
  \brief A solution to the map merging problem.
 
  It represents both a scored rototranslation.
 */
struct transformation {
  /*! Translation along x (columns) */
  int deltax;
  /*! Translation along y (rows) */
  int deltay;
  /*! Counterclockwise rotation  in degrees */
  int rotation;
  /*! Score */
  float ai;
  /*! Overlapping  value obtained -- end users do not need to use this */
  float overlapping;
};

/*!
  \brief Solves the merge map problem.
 
  This is probably the only function that the end user would call in a real world application.
  @param map1 first grid_map
  @param map2 second grid_map
  @param n_hypothesis number of hypothesis to produce
  @param hough_increment how many elements to should be skipped (see compute_Hough_transform for details)
  @param bool whether the randomized Hough transform should be used or not
  @param fraction fraction of points to use if the randomized version is used (must be between 0 and 1)
  @return a vector of hypothesis sorted in decreasing order accordingly to the score value. The vector will have at most n_hypothesis elements
 */
std::vector<transformation> get_hypothesis(const grid_map& map1,const grid_map& map2,unsigned int n_hypothesis,unsigned int hough_increment, bool randomized=false,float fraction=0.5);

/*!
  \brief Solves the merge map problem using a robust approach.
 
  This is probably the only function that the end user would call in a real world application. Additional robustness
is achieved by trying additional two rotations around each promising rotation identified in the Hough spectrum.
  @param map1 first grid_map
  @param map2 second grid_map
  @param n_hypothesis number of hypothesis to produce
  @param hough_increment how many elements to should be skipped (see compute_Hough_transform for details)
  @param bool whether the randomized Hough transform should be used or not
  @param fraction fraction of points to use if the randomized version is used (must be between 0 and 1)
  @return a vector of hypothesis sorted in decreasing order accordingly to the score value. The vector will have at most 3* n_hypothesis elements
 */
std::vector<transformation> get_hypothesis_robust(const grid_map&,const grid_map&,unsigned int,unsigned int, bool=false,float=0.5);

//void merge_maps(grid_map&,const grid_map&,const grid_map&,const transformation);

/*! 
  Computes the agreement between two maps, i.e. the number of cell that are free or occupied  in both maps
  @param m1 first map
  @param m2 second map
  @return agreement value
 */
unsigned int agreement(const grid_map& m1,const grid_map& m2);

/*! 
  Computes the agreement between two maps, i.e. the number of cell that are free in the first and
  occupied  in the second or viceversa.
  @param m1 first map
  @param m2 second map
  @return disagreement value
 */
unsigned int disagreement(const grid_map&,const grid_map&);

/*! 
  Computes the overlapping between two maps, i.e. the number of cell that are equal in both maps.
  @param m1 first map
  @param m2 second map
  @return overlapping value
 */
unsigned int overlapping(const grid_map&,const grid_map&);


/*! 
  Computes the acceptance index between two maps, i.e. the ration between agreement and the sum
between agreement and disagreement
  @param m1 first map
  @param m2 second map
  @return acceptance index
 */
float acceptance_index(const grid_map&,const grid_map&);

}

#endif
