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

#ifndef GRID_MAP
#define GRID_MAP

#include <vector>

#include "common.h"

/*!
\file grid_map.h
\brief Declaration of the grid_map class 
 */

/*!
\brief Namespace grouping all functions and classes needed by the merging algorithms.
 */
namespace mapmerge {

  /*!
    \brief Representation of an occupancy grid map

    The status of each cell is represented by an unsigned char. 
    Cells may be free, occupied or unknown. By default free is 255,
    occupied is 0 and unknown is 127.
   */
class grid_map {   
  
 private:
  
  unsigned int rows;
  unsigned int cols;
  unsigned char free_cell,occupied_cell,unknown_cell;
  

public:
  
  /*!
    Creates an empty grid map with standard size (to be resized later on).
  
   */
  grid_map();
  /*!
    Creates a  grid map with given size
    @param r: number of rows
    @param c: number of columns
   */
  grid_map(unsigned int r,unsigned int c);
  //~grid_map(); 
  
  /*! Returns the number of rows in the map */ 
  unsigned int get_rows() const { return rows; }
  /*! Returns the number of columns in the map */ 
  unsigned int get_cols() const { return cols; }
    /*! Returns the value used to encode free cells */ 
  unsigned char get_free_cell() const { return free_cell; }
  /*! Returns the value used to encode occupied cells */ 
  unsigned char get_occupied_cell() const { return occupied_cell; }
 /*! Returns the value used to encode unknown cells */
  unsigned char get_unknown_cell() const { return unknown_cell; }
  /*! Sets the value used to encode free cells 
     @param f new value for free cells
  */
  void set_free_cell(unsigned char f) { free_cell = f; };

 /*! Sets the value used to encode occupied cells 
     @param o new value for occupied cells
  */
  void set_occupied_cell(unsigned char o) { occupied_cell = o; };

 /*! Sets the value used to encode unknown cells 
     @param u new value for free cells
  */
  void set_unknown_cell(unsigned char u ) { unknown_cell = u; };
  
  /*!
    Copies the coordinates of all occupied grid cells into v. 
    Vector v is accordingly resized and its former content lost
    @param v vector where to copy the result
   */
  void get_points(std::vector<point>& v) const;
  /*! Resizes the grid map. Former contents are lost 
    @param r new number of rows
    @param c new number of columns
   */
  void resize_map(unsigned int r,unsigned int c);
  
  /*!
    Loads a grid map from an ascii file. 
    Values in the file must be organized into a table with space separated values. The grid map
    is accordingly resized and its former contents lost
    @param r number of rows
    @param c number of columns
    @param fn file name
    @return 0 if correctly loaded, 1 if the file could not be opened
   */
  int load_map(unsigned int r,unsigned int c,const char*fn);

  /*!
    Saves the grid map into an ascii file.
    Values are stored in an ascii table with space separated values
    @param fn: file name
    @return 0 if correctly saved, 1 if the file could not be opened
   */
  int save_map(const char *fn);
  
  /*!
    This actually stores the grid_cell. 
    It is public for efficiency reasons, so that you can read/write its values
    without getting through getter/setter methods. You should never resize this
    structure directly, but rather use the resize method.
   */
  std::vector<std::vector<unsigned int> > grid; // ugly, but only for efficiency reasons
  
};

}

#endif
