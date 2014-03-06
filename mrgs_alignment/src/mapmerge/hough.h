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

/*!
  \file hough.h
  \brief Routines to compute Hough transforms and related concepts
 */

#ifndef HOUGH_H
#define HOUGH_H

#include <vector>
#include "common.h"
#include "grid_map.h"

namespace mapmerge {

  /*!
    Returns the maximum row or column coordinate in a vector of points. Useful to determine rho
   */
unsigned int find_max_coordinate(const std::vector<point>&v);

/*!
Computes the Hough transform using a randomized approch. This is one of the improvements described
in the IROS 2008 paper.
@paramx HT grid_map where the result is stored (will be rescaled to rhosub rows and thetasub) columns
@param v vector of points with  occupied cells coordinates
@param thetasub number of subdivisions for the angle resolution
@param thosub number of subdivisions for resolution in rho
@param rhof  size of one step in rho
@fraction fraction of points to be considered when comuting the transform (must be between 0 and 1). Points
are randomly selected with a uniform distribution over v
*/
void compute_Randomized_Hough_transform(grid_map& HT,const std::vector<point>&v,unsigned int thetasub, unsigned int rhosub,float rhof,float fraction);

/*!
Computes the Hough transform using a deterministic approach. This is the basic method described in the
Autonomous Robots 2008 paper.
@paramx HT grid_map where the result is stored (will be rescaled to rhosub rows and thetasub) columns
@param v vector of points with  occupied cells coordinates
@param thetasub number of subdivisions for the angle resolution
@param thosub number of subdivisions for resolution in rho
@param rhof  size of one step in rho
@hough_increment determines how many points will be deterministically ignored. With 1 all points will be
taken, with 2 every second point will be taken (first, third, etc), with 3 every third point and so on.
Useful to speed up the computation but with an obvious tradeoff with accuracy.
*/
void compute_Hough_transform(grid_map& HT,const std::vector<point>& v,unsigned int thetasub, unsigned int rhosub, float rhof,unsigned int hough_increment);

/*!
  Computes the Hough spectrum of a previously computed Hough transform
  @param HT previously computed Hough transform
  @param HS computed Hough spectrum. The vector is appropriately resized and its former content lost
 */
void compute_Hough_spectrum(const grid_map& HT,std::vector<float>& HS);

/*!
  Computes the circular cross correlation between two vectors. 
  Important: the two input vectors must have the same size. 
  @param result computed circular cross correlation. Will be resized to the same size of the inputs. Normalized to 0-1 at the end
  @param a first input vector
  @param b second input vector
 */
void circular_cross_correlation(std::vector<float>& result,const std::vector<float>& a,
				const std::vector<float>& b);

/*!
  Finds the positions of a given number of local maxima in a vector using a circular approach, i.e. the vector warps around itself. 
@param maxima positions of the maxima (will have at most nh elements)
@param v vector where maxima should be sought
@param nh maximum number of maxima to be returned
 */
void find_local_maxima_circular(std::vector<unsigned int>& maxima,const std::vector<float>& v,unsigned int nh);

/*!
Computes the cross correlation between two vectors
@param result computed cross correlation. Will be appropriately resized (i.e. size of v1 + size of v2 - 1). Normalized to 0-1 at the end
@param v1 first input vector
@param v2 second input vector
 */
void cross_correlation(std::vector<float>& result,
		       const std::vector<float>& v1,const std::vector<float>&v2);


/*!
  Extracts the projection of a given map along one of the two axis, as described in the Autonomous Robots
paper.
@param out computed projection. Will be resized accordingly to the number of rows or columns of inmap, as 
prescribed by direction. Resul is normalized at the end (i.e. in the range 0-1)
@param inmap map from which the projection should be extracted
@param direction specifies the direction along which the projections should be computed. If 'X' or 'x' it will project alon the columns. Any other value will project along the rows 
 */
void extract_spectrum_xy(std::vector<float>& out,const grid_map& inmap,const char direction);

/*!
Extracts prjections along the axis in an improved way as described in the IROS 2008 paper. 
Note that this function processes a vector of points rather than a map, so the dimensions of the map
should be passed as additional parameters.
@param out computed projection. Will be resized accordingly to the number of rows or columns of inmap, as 
prescribed by direction. Resul is normalized at the end (i.e. in the range 0-1)
@param p vector of coordinates of occupied points
@param direction specifies the direction along which the projections should be computed. If 'X' or 'x' it will project alon the columns. Any other value will project along the rows.
@param r number of row in the original map 
@param c number of columns in the original map
@param increment
@param randomized specifies whether a radomized computation should be computed or not (defaults to false)
@param fraction fraction of points to be considered in the randomized case (defaults to 1)
 */
void extract_spectrum_xy_fast(std::vector<float>& out, const std::vector<point> p,const char direction,unsigned int r,unsigned int c,unsigned int increment=1,bool randomized=false,float fraction=1);

//void init_Hough_tables(unsigned int);


}

#endif
