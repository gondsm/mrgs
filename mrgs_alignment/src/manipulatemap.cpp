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

#include "manipulatemap.h"
#include "hough.h"
#include "io.h"

#include <cmath>
#include <cassert>
#include <algorithm>

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iterator>

using namespace std;

namespace mapmerge {

static const int theta_cells = 360;
static const int rho_cells = 256;
static const int THRESHOLD = 25;

float deg_to_rad(float d) {

  return d * M_PI / 180;

}

template<class T>
void generic_save(vector<T> v,const char* fname) 
{
     ofstream of(fname);
    
     ostream_iterator<T> output(of, "\n");
     copy(v.begin(),v.end(),output);

}


void print_matrix(const CvMat *m,const char * s) {

  cout << "Printing " << s << endl;
  cout << cvmGet(m,0,0) << " " << cvmGet(m,0,1) << " " << cvmGet(m,0,2) << endl;
  cout << cvmGet(m,1,0) << " " << cvmGet(m,1,1) << " " << cvmGet(m,1,2) << endl;
  cout << cvmGet(m,2,0) << " " << cvmGet(m,2,1) << " " << cvmGet(m,2,2) << endl;

}

void restore_map(grid_map& m) {

  unsigned int i,j,r = m.get_rows(), c = m.get_cols();
  unsigned char free_cell = m.get_free_cell(), 
    occupied_cell = m.get_occupied_cell(), 
    unknown_cell = m.get_unknown_cell();

  unsigned int low_threshold = unknown_cell - THRESHOLD;
  unsigned int high_threshold = unknown_cell + THRESHOLD;
 

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      if ( m.grid[i][j] < low_threshold )
	m.grid[i][j] = occupied_cell;
      else if ( m.grid[i][j] > high_threshold )
	m.grid[i][j] = free_cell;
      else 
	m.grid[i][j] = unknown_cell;
    }

  

}

// TESTED: OK
void cast_image_bw(grid_map& out,const grid_map& in) {

    int i,j;
    int r = in.get_rows() , c = in.get_cols();
    out.resize_map(r,c);
    unsigned char free_cell = in.get_free_cell(),
	occupied_cell = in.get_occupied_cell();

    for ( i = 0 ; i < r ; i++ )
	for ( j = 0 ; j < c ; j++ ) {
	    if ( in.grid[i][j] == occupied_cell )
		out.grid[i][j] = occupied_cell;
	    else
		out.grid[i][j]= free_cell;
	}

}

void translate_map(grid_map& newIm,const grid_map& oldim,
		  int dx,int dy) {

  int r  = oldim.get_rows(), c = oldim.get_cols(),i,j;
  int tr,tc;
  newIm.resize_map(r,c);

  //cout << "r = " << r << " c = " << c << endl;

  unsigned char background = newIm.get_unknown_cell();

  for ( i = 0 ; i < r ; i++ )
    fill(newIm.grid[i].begin(),newIm.grid[i].end(),background);

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      tr = i + dy;
      tc = j + dx;
      if ( ( tr >= 0 ) && ( tr < r ) && ( tc >= 0 ) && ( tc < c ) )
	newIm.grid[i][j] = oldim.grid[tr][tc];
    }

}



void overlap_maps(grid_map& out,const grid_map& m1,const grid_map& m2) {

  unsigned int r= m1.get_rows(),c = m1.get_cols(), i,j,
    free_cell = m2.get_free_cell(), occupied_cell = m2.get_occupied_cell(),
    unknown_cell = m2.get_unknown_cell();
  out.resize_map(r,c);

  out = m1;

  for ( i = 0 ; i < r ; i++ )
      for ( j = 0 ; j < c ; j++ ) {
	/* if ( ( m2.grid[i][j] == free_cell ) ||
	       ( m2.grid[i][j] == occupied_cell ) )
	       out.grid[i][j] = m2.grid[i][j]; */
	if (  m2.grid[i][j] == occupied_cell  )
	  out.grid[i][j] = occupied_cell;
	else if ( ( m2.grid[i][j] == free_cell ) && ( out.grid[i][j] == unknown_cell ) )
	  out.grid[i][j] = free_cell;
      }

}

 /* TESTED:OK */
void rotate_map(grid_map& outmap,const grid_map& inmap,int thetaDeg,unsigned char filler,float &outtx,float& outty) {
  
  //timeval tp1,tp2;
  //gettimeofday(&tp1,NULL);

  unsigned int i,j,r = inmap.get_rows(), c = inmap.get_cols();

  IplImage* img = cvCreateImage(cvSize(c,r), 8, 1);

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      ((uchar*)(img->imageData + img->widthStep*i))[j] = inmap.grid[i][j];
    }
  
  IplImage* dst = cvCreateImage(cvSize(c,r), 8, 1);

  CvMat *mat = cvCreateMat(2,3,CV_32FC1);
  cvSetIdentity(mat);

  float theta = deg_to_rad(thetaDeg);
  
  float tmp_c = ((float)(c) + 1) / 2;
  float tmp_r = ((float)(r) + 1) / 2;
  // implements a rotation about the center point
   outtx = tmp_c - cos(theta)*tmp_c + sin(theta)*tmp_r;
   outty = tmp_r - sin(theta)*tmp_c - cos(theta)*tmp_r;

  cvmSet(mat, 0, 2, outtx);
  cvmSet(mat, 1, 2, outty);
  cvmSet(mat,0,0,cos(theta));
  cvmSet(mat,1,1,cos(theta));
  cvmSet(mat,0,1,-sin(theta));
  cvmSet(mat,1,0,sin(theta));


   cvWarpAffine(img,dst,mat,
	       CV_INTER_CUBIC + CV_WARP_FILL_OUTLIERS,
	       cvScalarAll(filler));

  outmap.resize_map(r,c);

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      outmap.grid[i][j] = ((uchar*)(dst->imageData + dst->widthStep*i))[j];
    }

  // cvSaveImage("img.jpg",img);
  //cvSaveImage("dst.jpg",dst);
  cvReleaseImage(&img);
  cvReleaseImage(&dst);
  cvReleaseMat(&mat);

  //gettimeofday(&tp2,NULL);

  // cout << "\nTIME TO ROTATE " << compute_time(tp1,tp2) << endl;

}

// theta must be in degrees
void raw_transform_map(grid_map& outmap,const grid_map& inmap,float thetaDeg,float tx,float ty,unsigned char filler) {
  
  unsigned int i,j,r = inmap.get_rows(), c = inmap.get_cols();

  IplImage* img = cvCreateImage(cvSize(c,r), 8, 1);

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      ((uchar*)(img->imageData + img->widthStep*i))[j] = inmap.grid[i][j];
    }
  
  IplImage* dst = cvCreateImage(cvSize(c,r), 8, 1);

  CvMat *mat = cvCreateMat(2,3,CV_32FC1);
  cvSetIdentity(mat);

  float theta = deg_to_rad(thetaDeg);

  cvmSet(mat, 0, 2, tx);
  cvmSet(mat, 1, 2, ty);
  cvmSet(mat,0,0,cos(theta));
  cvmSet(mat,1,1,cos(theta));
  cvmSet(mat,0,1,-sin(theta));
  cvmSet(mat,1,0,sin(theta));

  cvWarpAffine(img,dst,mat,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,
	       cvScalarAll(filler));

  outmap.resize_map(r,c);

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      outmap.grid[i][j] = ((uchar*)(dst->imageData + dst->widthStep*i))[j];
    }

  // cvSaveImage("imgraw.jpg",img);
  //cvSaveImage("dstraw.jpg",dst);
  cvReleaseImage(&img);
  cvReleaseImage(&dst);
  cvReleaseMat(&mat);

}

// TESTED: OK
void save_map_to_file(const grid_map& inmap,const char*fn) {

  unsigned int i,j,r= inmap.get_rows(),c = inmap.get_cols();
  
  
  IplImage* img = cvCreateImage(cvSize(c,r), 8, 1);
  
  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      ((uchar*)(img->imageData + img->widthStep*i))[j] = inmap.grid[i][j];
    }

  cvSaveImage(fn,img);
  cvReleaseImage(&img);


}


ostream& operator<<(ostream& os,transformation& t) {

    os << t.deltax << " " << t.deltay << " " << t.rotation << " " << t.ai;
    return os;

}

static int my_compare(const transformation a,const transformation b) {

  return a.ai > b.ai;

}



// This is the more robust (but also slower) version 
vector<transformation> get_hypothesis_robust(const grid_map& map0,
					     const grid_map& map2,
					     unsigned int n_hypothesis,
					     unsigned int hough_increment,
					     bool randomized,
					     float fraction) 
{
  
  cout << "THIS ROUBST VERSION DOES NOT USE THE FAST SPECTRUM ACCELERATIONS" << endl;

  unsigned int i;
  vector<transformation> hypothesis;
  hypothesis.resize(3*n_hypothesis);

  //timeval tp1,tp2;
  //gettimeofday(&tp1,NULL);

  grid_map g0,g1,g2,map1;
  cast_image_bw(g0,map0);
  cast_image_bw(g2,map2);

  vector<point> p0,p1,p2;
  g0.get_points(p0);
  g2.get_points(p2);

  float rhoscale1,rhoscale2,rhoscale;

 

  rhoscale1 = find_max_coordinate(p0);
  rhoscale2 = find_max_coordinate(p2);

  rhoscale = 2 * max(rhoscale1,rhoscale2);

  grid_map HT0,HT1,HT2;

  //  init_Hough_tables(theta_cells);

  if ( randomized ) {
    compute_Randomized_Hough_transform(HT0,p0,theta_cells,rho_cells,rhoscale,fraction);
    compute_Randomized_Hough_transform(HT2,p2,theta_cells,rho_cells,rhoscale,fraction);
  }
  else {
    compute_Hough_transform(HT0,p0,theta_cells,rho_cells,rhoscale,hough_increment);
    compute_Hough_transform(HT2,p2,theta_cells,rho_cells,rhoscale,hough_increment);
  }

  //gettimeofday(&tp2,NULL);

 //cout << "\nTIME TO COMPUTE HT " << compute_time(tp1,tp2) << endl;

  //HT0.save_map("cHT0.txt");
  //HT2.save_map("cHT2.txt");

  vector<float> HTSpectrum0,HTSpectrum1,HTSpectrum2,CrossSpectra;
  compute_Hough_spectrum(HT0,HTSpectrum0);
  
  float outtx,outty;
  
  int adjustment = max_element(HTSpectrum0.begin(),HTSpectrum0.end())-HTSpectrum0.begin();
  float adjustment_rotation = float(adjustment) * 360 / theta_cells;
  //rotate_map(g1,g0,(int)round(adjustment_rotation),g1.get_free_cell(),outtx,outty);
  rotate_map(map1,map0,(int)adjustment_rotation,map1.get_unknown_cell(),outtx,outty);
  restore_map(map1);
  cast_image_bw(g1,map1);
  /* Needed for backing ups transormations later on */
  CvMat *T1 = cvCreateMat(3,3,CV_32FC1);
  CvMat *T1inv = cvCreateMat(3,3,CV_32FC1);
  CvMat *T3 = cvCreateMat(3,3,CV_32FC1);
  CvMat *T4 = cvCreateMat(3,3,CV_32FC1);
  CvMat *T5 = cvCreateMat(3,3,CV_32FC1);
  cvSetIdentity(T1);
  cvSetIdentity(T3);
  cvSetIdentity(T4);
  cvSetIdentity(T5);

  cvmSet(T1, 0, 2, outtx);
  cvmSet(T1, 1, 2, outty);
  cvmSet(T1,0,0,cos(deg_to_rad(adjustment_rotation)));
  cvmSet(T1,1,1,cos(deg_to_rad(adjustment_rotation)));
  cvmSet(T1,0,1,-sin(deg_to_rad(adjustment_rotation)));
  cvmSet(T1,1,0,sin(deg_to_rad(adjustment_rotation)));

  cvInvert(T1,T1inv);

  //print_matrix(T1,"T1");
  //print_matrix(T1inv,"T1inv");

  cvMatMul(T1,T1inv,T5);

  //print_matrix(T5,"T5");

  g1.get_points(p1);

  if ( randomized )
    compute_Randomized_Hough_transform(HT1,p1,theta_cells,rho_cells,rhoscale,fraction);
  else
    compute_Hough_transform(HT1,p1,theta_cells,rho_cells,rhoscale,hough_increment);
  
  //compute_Hough_transform(HT1,p1,theta_cells,rho_cells,rhoscale,hough_increment);

  compute_Hough_spectrum(HT1,HTSpectrum1);
  compute_Hough_spectrum(HT2,HTSpectrum2);

  circular_cross_correlation(CrossSpectra,HTSpectrum1,HTSpectrum2);

  vector<unsigned int> maxima(n_hypothesis);
  find_local_maxima_circular(maxima,CrossSpectra,n_hypothesis);

  vector<int> rotationEstimatePrel(n_hypothesis);
  for (  i = 0 ; i < n_hypothesis ; i++ )
    rotationEstimatePrel[i] = (int)round((float)(maxima[i]) * 360 / theta_cells);


  vector<int> rotationEstimate(n_hypothesis*3);

  for ( i = 0 ; i < n_hypothesis ; i++ ) {
    rotationEstimate[3*i] = rotationEstimatePrel[i];
    rotationEstimate[3*i+1] = rotationEstimatePrel[i]+1;
    rotationEstimate[3*i+2] = rotationEstimatePrel[i]-1;
  }

  n_hypothesis *= 3;

  grid_map imrot,intermediatemap,intermediatemap2,imtrans,resultmap;
  vector<float> XSpectrum1,YSpectrum1,XSpectrum2,YSpectrum2;
  vector<float> crossX,crossY;
  int mx,my,deltax,deltay;
  transformation tmp_trans;

  //  char name[40];// = { "result1.png" , "result2.png" , "result3.png" , "result4.png" };

  extract_spectrum_xy(XSpectrum1,g1,'X');
  extract_spectrum_xy(YSpectrum1,g1,'Y');
    

  unsigned int hyp;
  for ( hyp = 0 ; hyp < n_hypothesis ; hyp++ ) {

    //    cout << "INTERNAL ESTIMATE " << rotationEstimate[hyp] <<  " " << flush;



    rotate_map(imrot,g2,-rotationEstimate[hyp],g2.get_free_cell(),outtx,outty);

    //restore_map(imrot);
    
 
    extract_spectrum_xy(XSpectrum2,imrot,'X');
    extract_spectrum_xy(YSpectrum2,imrot,'Y');

    cross_correlation(crossX,XSpectrum1,XSpectrum2);
    cross_correlation(crossY,YSpectrum1,YSpectrum2);
    
    mx = max_element(crossX.begin(),crossX.end())-crossX.begin()+1;
    my = max_element(crossY.begin(),crossY.end())-crossY.begin()+1;

    //cout << "mx =  " << mx << " my = " << my<< endl;
    
    deltax = crossX.size()+1-XSpectrum1.size()-mx;
    deltay = crossY.size()+1-YSpectrum1.size()-my;


    cvmSet(T3,0,0,cos(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,1,1,cos(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,0,1,-sin(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,1,0,sin(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,0,2,outtx-deltax);
    cvmSet(T3,1,2,outty-deltay);
      
    cvMatMul(T1inv,T3,T4);

    raw_transform_map(imtrans,map2,-rotationEstimate[hyp]-adjustment_rotation,cvmGet(T4,0,2),cvmGet(T4,1,2),map2.get_unknown_cell());

    restore_map(imtrans);
    // now recover the transformation in the original frame
    
    float rot_original = -rotationEstimate[hyp] - adjustment_rotation;
    
    tmp_trans.deltax = (int)deltax;
    tmp_trans.deltay = (int)deltay;
    tmp_trans.rotation = (int)rot_original;
    tmp_trans.ai = acceptance_index(map0,imtrans); 
    tmp_trans.overlapping = overlapping(map0,imtrans)/(map0.get_rows()*map0.get_cols()); 
    
    overlap_maps(resultmap,map0,imtrans);
    
    //restore_map(map1);
    restore_map(resultmap);

    //    if ( ( ! check_map(imtrans) ) || ( ! check_map(map1) ) || ( ! check_map(resultmap) ) )
    // cout << "Map anomaly"<<endl;

    /*    
    sprintf(name,"result%d.png",hyp);

    save_map_to_file(imrot,"imrot.jpg");
    save_map_to_file(intermediatemap,"interm.jpg");
    save_map_to_file(map0,"m0.jpg");
    
    save_map_to_file(map2,"m2.jpg");
    save_map_to_file(resultmap,name);
    save_map_to_file(map1,"rotated_map.jpg");
    save_map_to_file(imrot,"intermediate.jpg");
    
    generic_save(HTSpectrum0,"Spectrum0.txt");
    generic_save(HTSpectrum1,"Spectrum1.txt");
    generic_save(HTSpectrum2,"Spectrum2.txt");
    generic_save(CrossSpectra,"CrossSpectra.txt");
    generic_save(maxima,"maxima.txt");
    generic_save(XSpectrum1,"xspectrum1.txt");
    generic_save(YSpectrum1,"yspectrum1.txt");
    generic_save(XSpectrum2,"xspectrum2.txt");
    generic_save(YSpectrum2,"yspectrum2.txt");
    generic_save(rotationEstimate,"rotationEstimate.txt");
    generic_save(crossX,"CrossCorrelationX.txt");
    generic_save(crossY,"CrossCorrelationY.txt");
    */

    hypothesis[hyp] = tmp_trans;
    
    //  cout << tmp_trans << endl;
  }


  
  cvReleaseMat(&T1);
  cvReleaseMat(&T1inv);
  cvReleaseMat(&T3);
  cvReleaseMat(&T4);
  cvReleaseMat(&T5);

  sort(hypothesis.begin(),hypothesis.end(),my_compare);
  
  return hypothesis;
}


// This is the basic  version 
vector<transformation> get_hypothesis(const grid_map& map0,
				      const grid_map& map2,
				      unsigned int n_hypothesis,
				      unsigned int hough_increment,
				      bool randomized,
				      float fraction) 
{
  

  unsigned int i;
  vector<transformation> hypothesis;
  hypothesis.resize(n_hypothesis);

  //timeval tp1,tp2;
  //gettimeofday(&tp1,NULL);

  grid_map g0,g1,g2,map1;
  cast_image_bw(g0,map0);
  cast_image_bw(g2,map2);

  vector<point> p0,p1,p2;
  g0.get_points(p0);
  g2.get_points(p2);

  float rhoscale1,rhoscale2,rhoscale;

 

  rhoscale1 = find_max_coordinate(p0);
  rhoscale2 = find_max_coordinate(p2);

  rhoscale = 2 * max(rhoscale1,rhoscale2);

  grid_map HT0,HT1,HT2;

  //  init_Hough_tables(theta_cells);

  if ( randomized ) {
    compute_Randomized_Hough_transform(HT0,p0,theta_cells,rho_cells,rhoscale,fraction);
    compute_Randomized_Hough_transform(HT2,p2,theta_cells,rho_cells,rhoscale,fraction);
  }
  else {
    compute_Hough_transform(HT0,p0,theta_cells,rho_cells,rhoscale,hough_increment);
    compute_Hough_transform(HT2,p2,theta_cells,rho_cells,rhoscale,hough_increment);
  }
  
  vector<float> HTSpectrum0,HTSpectrum1,HTSpectrum2,CrossSpectra;
  compute_Hough_spectrum(HT0,HTSpectrum0);
  
  float outtx,outty;
  
  int adjustment = max_element(HTSpectrum0.begin(),HTSpectrum0.end())-HTSpectrum0.begin();
  float adjustment_rotation = float(adjustment) * 360 / theta_cells;
  //rotate_map(g1,g0,(int)round(adjustment_rotation),g1.get_free_cell(),outtx,outty);
  rotate_map(map1,map0,(int)adjustment_rotation,map1.get_unknown_cell(),outtx,outty);
  restore_map(map1);
  cast_image_bw(g1,map1);

  /* Needed for backing ups transormations later on */
  CvMat *T1 = cvCreateMat(3,3,CV_32FC1);
  CvMat *T1inv = cvCreateMat(3,3,CV_32FC1);
  CvMat *T3 = cvCreateMat(3,3,CV_32FC1);
  CvMat *T4 = cvCreateMat(3,3,CV_32FC1);
  CvMat *T5 = cvCreateMat(3,3,CV_32FC1);
  cvSetIdentity(T1);
  cvSetIdentity(T3);
  cvSetIdentity(T4);
  cvSetIdentity(T5);

  cvmSet(T1, 0, 2, outtx);
  cvmSet(T1, 1, 2, outty);
  cvmSet(T1,0,0,cos(deg_to_rad(adjustment_rotation)));
  cvmSet(T1,1,1,cos(deg_to_rad(adjustment_rotation)));
  cvmSet(T1,0,1,-sin(deg_to_rad(adjustment_rotation)));
  cvmSet(T1,1,0,sin(deg_to_rad(adjustment_rotation)));

  cvInvert(T1,T1inv);

  //print_matrix(T1,"T1");
  //print_matrix(T1inv,"T1inv");

  cvMatMul(T1,T1inv,T5);

  //print_matrix(T5,"T5");

  g1.get_points(p1);
  if ( randomized )
    compute_Randomized_Hough_transform(HT1,p1,theta_cells,rho_cells,rhoscale,fraction);
    else
      compute_Hough_transform(HT1,p1,theta_cells,rho_cells,rhoscale,hough_increment);

  compute_Hough_spectrum(HT1,HTSpectrum1);
  compute_Hough_spectrum(HT2,HTSpectrum2);

  circular_cross_correlation(CrossSpectra,HTSpectrum1,HTSpectrum2);

  vector<unsigned int> maxima(n_hypothesis);
  find_local_maxima_circular(maxima,CrossSpectra,n_hypothesis);

  vector<int> rotationEstimate(n_hypothesis);
  for (  i = 0 ; i < n_hypothesis ; i++ )
    rotationEstimate[i] = (int)round((float)(maxima[i]) * 360 / theta_cells);


  grid_map imrot,intermediatemap,intermediatemap2,imtrans,resultmap;
  vector<float> XSpectrum1,YSpectrum1,XSpectrum2,YSpectrum2,XSpectrumFast,YSpectrumFast;
  vector<float> crossX,crossY;
  int mx,my,deltax,deltay;
  transformation tmp_trans;

  //  char name[40];// = { "result1.png" , "result2.png" , "result3.png" , "result4.png" };

  if ( randomized ) {
    extract_spectrum_xy_fast(XSpectrum1,p1,'X',map1.get_rows(),map1.get_cols(),hough_increment,true,fraction);
    extract_spectrum_xy_fast(YSpectrum1,p1,'Y',map1.get_rows(),map1.get_cols(),hough_increment,true,fraction);    
  }
  
  else {
    extract_spectrum_xy_fast(XSpectrum1,p1,'X',map1.get_rows(),map1.get_cols(),hough_increment,false,fraction);
    extract_spectrum_xy_fast(YSpectrum1,p1,'Y',map1.get_rows(),map1.get_cols(),hough_increment,false,fraction);    
  }
  
  unsigned int hyp;
  for ( hyp = 0 ; hyp < n_hypothesis ; hyp++ ) {


    rotate_map(imrot,g2,-rotationEstimate[hyp],g2.get_free_cell(),outtx,outty);

    // here it makes no sense using the fast version because in order to extract the point I pay anyway complexity O(rc) 
    extract_spectrum_xy(XSpectrum2,imrot,'X');
    extract_spectrum_xy(YSpectrum2,imrot,'Y');
  
    cross_correlation(crossX,XSpectrum1,XSpectrum2);
    cross_correlation(crossY,YSpectrum1,YSpectrum2);
    
    mx = max_element(crossX.begin(),crossX.end())-crossX.begin()+1;
    my = max_element(crossY.begin(),crossY.end())-crossY.begin()+1;

    deltax = crossX.size()+1-XSpectrum1.size()-mx;
    deltay = crossY.size()+1-YSpectrum1.size()-my;


    cvmSet(T3,0,0,cos(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,1,1,cos(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,0,1,-sin(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,1,0,sin(deg_to_rad(-rotationEstimate[hyp])));
    cvmSet(T3,0,2,outtx-deltax);
    cvmSet(T3,1,2,outty-deltay);
      
    cvMatMul(T1inv,T3,T4);

    raw_transform_map(imtrans,map2,-rotationEstimate[hyp]-adjustment_rotation,cvmGet(T4,0,2),cvmGet(T4,1,2),map2.get_unknown_cell());

    restore_map(imtrans);
    // now recover the transformation in the original frame
    
    float rot_original = -rotationEstimate[hyp] - adjustment_rotation;
    
    tmp_trans.deltax = (int)deltax;
    tmp_trans.deltay = (int)deltay;
    tmp_trans.rotation = (int)rot_original;
    tmp_trans.ai = acceptance_index(map0,imtrans); 
    tmp_trans.overlapping = overlapping(map0,imtrans)/(map0.get_rows()*map0.get_cols()); 
    
    overlap_maps(resultmap,map0,imtrans);
    
    //restore_map(map1);
    restore_map(resultmap);
    /*
    sprintf(name,"result%d.png",hyp);

    save_map_to_file(imrot,"imrot.jpg");
    save_map_to_file(intermediatemap,"interm.jpg");
    save_map_to_file(map0,"m0.jpg");
    
    save_map_to_file(map2,"m2.jpg");
    save_map_to_file(resultmap,name);
    save_map_to_file(map1,"rotated_map.jpg");
    save_map_to_file(imrot,"intermediate.jpg");
    
    generic_save(HTSpectrum0,"Spectrum0.txt");
    generic_save(HTSpectrum1,"Spectrum1.txt");
    generic_save(HTSpectrum2,"Spectrum2.txt");
    generic_save(CrossSpectra,"CrossSpectra.txt");
    generic_save(maxima,"maxima.txt");
    generic_save(XSpectrum1,"xspectrum1.txt");
    generic_save(YSpectrum1,"yspectrum1.txt");


    generic_save(XSpectrum2,"xspectrum2.txt");
    generic_save(YSpectrum2,"yspectrum2.txt");
    generic_save(rotationEstimate,"rotationEstimate.txt");
    generic_save(crossX,"CrossCorrelationX.txt");
    generic_save(crossY,"CrossCorrelationY.txt");
    
    */
    hypothesis[hyp] = tmp_trans;
    
  }


  
  cvReleaseMat(&T1);
  cvReleaseMat(&T1inv);
  cvReleaseMat(&T3);
  cvReleaseMat(&T4);
  cvReleaseMat(&T5);

  sort(hypothesis.begin(),hypothesis.end(),my_compare);
  
  return hypothesis;
}


bool check_map(const grid_map& m) {

  unsigned int free_cell = m.get_free_cell() , 
    occupied_cell = m.get_occupied_cell() ,
    unknown_cell = m.get_unknown_cell();

  unsigned int i,j,r = m.get_rows(), c = m.get_cols();

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ )
      if ( ( m.grid[i][j] != free_cell ) &&
	   ( m.grid[i][j] != occupied_cell ) &&
	   ( m.grid[i][j] != unknown_cell ) )
	return false;

  return true;
}

// SEEMS OK
unsigned int agreement(const grid_map& g1,const grid_map& g2) {

  unsigned int agr = 0;

  unsigned char free_cell,occupied_cell,unknown_cell;
  unsigned int r = g1.get_rows(), c = g1.get_cols();
  unsigned int i,j;

  free_cell = g1.get_free_cell();
  occupied_cell = g1.get_occupied_cell();
  unknown_cell = g1.get_unknown_cell();

  assert((g1.get_rows() == g2.get_rows()) &&
	 (g1.get_cols() == g2.get_cols()));

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      if ( ( g1.grid[i][j] == g2.grid[i][j] ) && 
	   ( g1.grid[i][j] != unknown_cell) )
	agr++;
    }

  return agr;

}




// SEEMS OK
unsigned int disagreement(const grid_map& g1,const grid_map& g2) {
  unsigned int disagr = 0;

  unsigned char free_cell,occupied_cell,unknown_cell;
  unsigned int r = g1.get_rows(), c = g1.get_cols();
  unsigned int i,j;

  free_cell = g1.get_free_cell();
  occupied_cell = g1.get_occupied_cell();
  unknown_cell = g1.get_unknown_cell();

  assert((g1.get_rows() == g2.get_rows()) &&
	 (g1.get_cols() == g2.get_cols()));

  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ ) {
      if ( ( g1.grid[i][j] != g2.grid[i][j] ) && 
	   ( g1.grid[i][j] != unknown_cell) &&
	   ( g2.grid[i][j] != unknown_cell ) )
	disagr++;
    }

  return disagr;

}

unsigned int overlapping(const  grid_map& g1,const grid_map&g2) {

  unsigned int r = g1.get_rows(), c = g1.get_cols(), i,j,retval = 0;
  
  for ( i = 0 ; i < r ; i++ )
    for ( j = 0 ; j < c ; j++ )
      if( g1.grid[i][j] == g2.grid[i][j] )
	retval++;

  return retval;

}

// OK
float acceptance_index(const grid_map&g1,const grid_map&g2) {
  unsigned int agr = agreement(g1,g2);
  unsigned int dis = disagreement(g1,g2);
  
  //cout << " Acceptance index : agr = " << agr << " dis " << dis<<endl;
  
    return float(agr) / ( agr + dis ); 
    //return ((float)overlapping(g1,g2))/(g1.get_rows()*g1.get_cols());
}


}
