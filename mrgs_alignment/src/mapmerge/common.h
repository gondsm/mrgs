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
  \file common.h
  \brief General purpose data types and functions  that do not fit anywhere else
 */

#ifndef COMMON_H
#define COMMON_H

#include <sys/time.h>

namespace mapmerge {

  /*!
    \brief Point in a grid cell map
   */
struct point {
  /*! Row position of the point */
  unsigned int r;
  /*! Column position of the point */
  unsigned int c;
};

/*!
  \fn unsigned long int compute_time(const timeval& start,const timeval& end);
  Computes the time elapsed between start and end (as returned by gettimeofday)
  @param start start time
  @param end end time
  @return time elapsed in milliseconds
 */
unsigned long int compute_time(const timeval& start,const timeval& end);

}

#endif
