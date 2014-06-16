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

#include "grid_map.h"


#include <stdlib.h>
#include <algorithm>
#include <fstream>
#include <iostream>

using namespace std;

namespace mapmerge {

grid_map::grid_map() {

  resize_map(10,10);
  
  set_free_cell(255);
  set_occupied_cell(0);
  set_unknown_cell(127);

}

grid_map::grid_map(unsigned int r,unsigned int c) {
  
  resize_map(r,c);
  
  set_free_cell(255);
  set_occupied_cell(0);
  set_unknown_cell(127);
  
}

void grid_map::resize_map(unsigned int r,unsigned int c) {

  rows = r;
  cols = c;
  grid.resize(r);
  for ( unsigned int i = 0 ; i < r  ; i++ ) {
    grid[i].resize(c);
    fill(grid[i].begin(),grid[i].end(),0);
  }
  
}


// TESTED OK
void grid_map::get_points(vector<point>& p) const {

  unsigned int np,i,j,pos;

  np = 0;
  for ( i = 0 ; i < rows ; i++ )
    np+=count(grid[i].begin(),grid[i].end(),occupied_cell); 
  
  p.resize(np);
  pos = 0;
  for ( i = 0 ; i < rows ; i++ )
    for ( j = 0 ; j < cols ; j++ )
      if ( grid[i][j] == occupied_cell ) {
	p[pos].r = i+1;
	p[pos].c = j+1;
	pos++;
      }
}


// TESTED: OK
int grid_map::load_map(unsigned int r,unsigned int c,const char* fname) {

  resize_map(r,c);

  ifstream ifs(fname);

  if ( ! ifs ) 
    return 1;
  
  unsigned int i,j;
  unsigned int tmp;
  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      ifs >> tmp;
      grid[i][j] = static_cast<unsigned int>(tmp);
    }
  ifs.close();

  return 0;

}

// TESTED: OK
int grid_map::save_map(const char *fname) {

  ofstream ofs(fname);
  if (! ofs ) 
    return 1;

  for ( unsigned int i = 0 ; i < rows ; i++ ) {
    for (unsigned int j = 0 ; j < cols ; j++ ) {
      ofs << static_cast<unsigned int>(grid[i][j]) << " ";
    }
    ofs << endl;
  }
  
  ofs.close();

  return 0;

}


}
