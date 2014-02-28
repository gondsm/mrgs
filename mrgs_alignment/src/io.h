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

/*! \file io.h
  \brief Simple io routines. Mainly needed for debugging purposes.
 */

#ifndef IO_H
#define IO_H

#include "common.h"
#include <vector>
#include <fstream>


namespace mapmerge {

/*! 
  \fn std::ostream& operator<<(std::ostream& os,const point& p);
  Overloaded output inserter operator 
  @param os: output stream
  @param p: point to print
  @return the output stream, as required by the operator 
    
*/
std::ostream& operator<<(std::ostream& os,const point& p);

/*! 
  \fn template<class T> int generic_save(std::vector<T>,const char*);
  Saves a templatized vector to a file in tabular form (one per line)
  << operator needs to be suitably overloaded
  @param v: vector to save
  @param f: file name
  @return 0 if success, 1 if it was not possible to open the file
 */
template<class T>
int generic_save(std::vector<T>,const char*);

}

#endif
