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

#include <climits>

#include <cmath>
#include <algorithm>
#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <cassert>
using namespace std;

namespace mapmerge {

const float EPSILON = 0.00001;



// TESTED: OK
unsigned int find_max_coordinate(const vector<point>& v) {

  unsigned int retval = max(v[0].r,v[0].c);

  vector<point>::const_iterator i;
  for( i =  v.begin(),  i++; i != v.end() ; i++ ) 
    retval = max((*i).c,(*i).r) > retval ? max((*i).c,(*i).r) : retval;

  return retval;
}

/* float *costable,*sintable;

void init_Hough_tables(unsigned int thetasub) {

  costable = new float[thetasub];
  sintable = new float[thetasub];

  unsigned int j;
  float thetainc  = 2* M_PI /thetasub,theta;

  theta = thetainc;
  for ( j = 0 ; j < thetasub; j++ ) {
    costable[j] = cos(theta);
    sintable[j] = sin(theta);
    theta += thetainc;
  }

}
*/


// TESTED: OK 
void compute_Hough_transform(grid_map& HT,const vector<point>& p,
			     unsigned int thetasub, 
			     unsigned int rhosub,
			     float rhof,
			     unsigned int hough_increment) {

  float thetainc = 2* M_PI /thetasub,theta,rho,rhos;
  HT.resize_map(rhosub,thetasub);
  unsigned int i,j,np;
  int rho_index;
  float costable[thetasub],sintable[thetasub];
  
  theta = thetainc;
  for ( j = 0 ; j < thetasub; j++ ) {
    costable[j] = cos(theta);
    sintable[j] = sin(theta);
    theta += thetainc;
  }
  
  
  for ( i = 0 , np = p.size() ; i < np ; i+=hough_increment ) {
    theta = thetainc;
    for ( j = 0 ; j < thetasub ; j++ ) {
	//rho = cos(theta)*p[i].r + sin(theta)*p[i].c;
      rho = costable[j]*p[i].r + sintable[j]*p[i].c;
      rho *= rhosub / rhof;
      rhos = rho+rhosub/2;
      rho = round(rho+rhosub/2);
           
      rho_index = static_cast<int>(rho);
      if ( ( rho_index >= 1 ) && (rho_index <= (int)rhosub) ) 
	HT.grid[rho_index-1][j]++;

      theta += thetainc;
      
    }
    
  }

} 


void compute_Randomized_Hough_transform(grid_map& HT,
					const vector<point>& p,
					unsigned int thetasub, 
					unsigned int rhosub,
					float rhof,
					float fraction) {

  float thetainc = 2* M_PI /thetasub,theta,rho,rhos;
  HT.resize_map(rhosub,thetasub);
  unsigned int i,j,np;
  int rho_index;
  float costable[thetasub],sintable[thetasub];
  
  theta = thetainc;
  for ( j = 0 ; j < thetasub; j++ ) {
    costable[j] = cos(theta);
    sintable[j] = sin(theta);
    theta += thetainc;
  }
  
  unsigned int s = p.size();
  unsigned int number_of_points = (unsigned int)round(s * fraction); 

  unsigned int *indexes = new unsigned int [number_of_points];
 
  
  for ( i = 0 ; i < number_of_points ; i++ ) 
    indexes[i] = (unsigned int)round(((float)random())/ INT_MAX * s);


 
  for ( i = 0 , np = p.size() ; i < number_of_points ; i++ ) {
    theta = thetainc;
    for ( j = 0 ; j < thetasub ; j++ ) {
	//rho = cos(theta)*p[i].r + sin(theta)*p[i].c;
      rho = costable[j]*p[indexes[i]].r + sintable[j]*p[indexes[i]].c;
      rho *= rhosub / rhof;
      rhos = rho+rhosub/2;
      rho = round(rho+rhosub/2);
           
      rho_index = static_cast<int>(rho);
      if ( ( rho_index >= 1 ) && (rho_index <= (int)rhosub) ) 
	HT.grid[rho_index-1][j]++;

      theta += thetainc;
      
    }
    
  }

  delete indexes;

} 

// TESTED: OK
void compute_Hough_spectrum(const grid_map& HT,vector<float>& HS) {

  HS.resize(HT.get_cols());
  fill(HS.begin(),HS.end(),0);
  unsigned int i,j,r,c;
  r = HT.get_rows();
  c = HT.get_cols();

  for ( i = 0 ; i < c ; i++ )
    for ( j = 0 ; j < r ; j++ )
      HS[i] += HT.grid[j][i]*HT.grid[j][i];


  float max = *(max_element(HS.begin(),HS.end()));

  for (  i = 0 ; i < HS.size() ; i++ ) 
    HS[i] /= max;

}

// TESTED: OK
void circular_cross_correlation(vector<float>& result,const vector<float>&a,
				 const vector<float>&b) {

  unsigned int i,j,k,s;

  assert(a.size() == b.size());
  s = a.size();
  result.resize(s);
  fill(result.begin(),result.end(),0);

  for ( i = 0 ; i < s ; i++ ) {
    k = i == 0 ? 0 : s - i;
    for ( j = 0  ; j < s ; j++ , ++k %= s )
      result[i] += a[j]*b[k];
  }
  

  float mi,ma;
  mi = *(min_element(result.begin(),result.end()));
  ma = *(max_element(result.begin(),result.end()));
	 
  for ( i = 0 ; i < s ; i++ )
    result[i] = (result[i] - mi ) / ( ma - mi );

}

bool operator<(const pair<unsigned int,float>& a,
	       const pair<unsigned int,float>& b) {

  if ( a.first < b.first )
    return false;
  else return true;

}

// this allows to sort in reversed order
int my_compare(pair<unsigned int,float> a,pair<unsigned int,float> b) 
{
    if ( a.second > b.second )
	return true;
    else return false;

}


// TESTED: OK
void find_local_maxima_circular(vector<unsigned int>& maxima,
				const vector<float>& seq,
				unsigned int nh) {

  unsigned int i,j, s= seq.size();
  vector<float> ex(s+2),m(s+2);

  ex[0] = seq[s-1];
  for ( i = 1 , j = 0 ; j < s ; j++ , i++ )
    ex[i] = seq[j];
  ex[s+1] = seq[0];

  for ( i= 1 ; i < s+1 ; i++ )
    m[i] = ( ( ex[i] > ex[i-1] ) && ( ex[i] > ex[i+1] ) ) ? 1 : 0;

  unsigned int n_max = count(m.begin(),m.end(),1);
  vector<pair<unsigned int,float> >max_pos(n_max);

  for ( i = 0 , j=0  ; i < s+2 ; i++ ) 
    if( m[i] == 1 ) {
      max_pos[j].first = i;
      max_pos[j].second = ex[i];
      j++;
    }

  // THIS NEEDS TO BE FIXED
  sort(max_pos.begin(),max_pos.end(),my_compare);


  if ( n_max < nh ) 
    nh = n_max;

  maxima.resize(nh);

  for ( i = 0 ; i < nh ; i++ )
    maxima[i] = max_pos[i].first - 1;

}

void extract_spectrum_xy_fast(vector<float>& out, 
			      const vector<point> p,
			      const char direction,
			      unsigned int r,unsigned int c,
			      unsigned int increment,bool randomized,float fraction) {

  unsigned int i,j,s = p.size();
  

  if ( ! randomized ) {
    
    if ( ( direction =='X' ) || ( direction =='x' ) ) {
      out.resize(c);
      fill(out.begin(),out.end(),0);
      for ( i = 0 ; i < s ; i+=increment )
	out[p[i].c-1]++;

    }
    else  {
      out.resize(r);
      fill(out.begin(),out.end(),0);
      for ( i = 0 ; i < s ; i+= increment )
	out[p[i].r-1]++;

    }
  }
  else { // ramdomized spectrum calculation
     unsigned int number_of_points = (unsigned int)round(s * fraction); 
     if ( ( direction =='X' ) || ( direction =='x' ) ) {
      out.resize(c);
      fill(out.begin(),out.end(),0);
      for ( i = 0 ; i < number_of_points ; i++ ) {
	j = (unsigned int)trunc(((float)random())/ INT_MAX * s);
	out[p[j].c-1]++;
      }

    }
    else  {
      out.resize(r);
      fill(out.begin(),out.end(),0);
      for ( i = 0 ; i < number_of_points ; i++ ) {
	j = (unsigned int)trunc(((float)random())/ INT_MAX * s);
	out[p[j].r-1]++;
      }

    }
  }
  
  
  
  float maxv = *max_element(out.begin(),out.end());
  j = out.size();
  for ( i = 0  ; i < j ; i++ )
    out[i] /= maxv;
  
  
}

// TESTED: OK
void extract_spectrum_xy(vector<float>& out,const grid_map&inmap,const char direction) 
{

    unsigned int i,j,rows,cols;
    rows = inmap.get_rows();
    cols = inmap.get_cols();
    /*
    size = p.size();

    assert((direction=='x')||(direction=='X')||(direction=='y')||(direction=='Y'));

    if ( ( direction =='X' ) || ( direction =='x' ) ) {
      out.resize(cols);
      fill(out.begin(),out.end(),0);
      for ( i = 0 ; i < size ; i++ )
	out[p[i].c]++;
	  
    }
    else  {
      out.resize(rows);
      fill(out.begin(),out.end(),0);
      for ( i = 0 ; i < size ; i++ )
	  out[p[i].r]++;;
    }
    */

    
    if ( ( direction =='X' ) || ( direction =='x' ) ) {
	out.resize(cols);
	fill(out.begin(),out.end(),0);
	for ( i = 0 ; i < cols ; i++ )
	    for ( j = 0 ; j < rows ; j++ )
		out[i] += ( 255 - inmap.grid[j][i]);
    }
    else  {
	out.resize(rows);
	fill(out.begin(),out.end(),0);
	for ( i = 0 ; i < rows ; i++ )
	    for ( j = 0 ; j < cols ; j++ )
		out[i] += ( 255 - inmap.grid[i][j]);
    }
 
    

    float maxv = *max_element(out.begin(),out.end());
    j = out.size();
    for ( i = 0  ; i < j ; i++ )
      out[i] /= maxv;

}


// TESTED: OK
void cross_correlation(vector<float>& out,
		       const vector<float>& v1,const vector<float>& v2) 
{

  int s1 = v1.size(), s2 = v2.size();  
  unsigned int outsize = s1 + s2 - 1;
  int tau,k;
  out.resize(outsize);
  fill(out.begin(),out.end(),0);

  for ( tau = -s2 + 1 ; tau < s1 ; tau++ ) {

    for ( k = 0 ; k < s2 ; k++ ) 
      if ( ( k+tau >= 0 ) && ( k + tau < s1 )  )
	   out[tau+s2-1] += v1[k+tau] * v2[k];

  }
  
  float max = *(max_element(out.begin(),out.end()));

  for ( unsigned int i = 0 ; i < outsize ; i++ )
    out[i] /=max;
  
}

}
