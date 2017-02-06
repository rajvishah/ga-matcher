/*
 * Author : Rajvi Shah (rajvi.a.shah@gmail.com)
 * SIFT-like feature matching implementation introduced in the following paper:
 *
 * "Geometry-aware Feature Matching for Structure from Motion Applications", 
 * Rajvi Shah, Vanshika Srivastava and P J Narayanan, WACV 2015.
 * http://researchweb.iiit.ac.in/~rajvi.shah/projects/multistagesfm/
 *
 *
 * Copyright (c) 2015 International Institute of Information Technology - 
 * Hyderabad 
 * All rights reserved.
 *   
 *  Permission to use, copy, modify and distribute this software and its 
 *  documentation for educational purpose is hereby granted without fee 
 *  provided that the above copyright notice and this permission notice 
 *  appear in all copies of this software and that you do not sell the software.
 *     
 *  THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND, 
 *  EXPRESSED, IMPLIED OR OTHERWISE.
 */

#include "defs.h"
#include "Geometric.h"

void geometry::ComputeRectangleEdges(double width, double height, 
    vector< vector< double > >& rectEdges) {

  double topLeft[2] = {0, 0};
  double topRight[2] = {width, 0};
  double bottomLeft[2] = {0, height};
  double bottomRight[2] = {width, height};
  

  
  /* NC Change
  double topLeft[2] = {-width/2, height/2};
  double topRight[2] = {width/2, height/2};
  double bottomLeft[2] = {-width/2, -height/2};
  double bottomRight[2] = {width/2, -height/2};
  */

  rectEdges.resize(4);
  rectEdges[0].resize(3,-1.0f);
  rectEdges[1].resize(3,-1.0f);
  rectEdges[2].resize(3,-1.0f);
  rectEdges[3].resize(3,-1.0f);
  vector<double>& line1 = rectEdges[0];
  vector<double>& line2 = rectEdges[1];
  vector<double>& line3 = rectEdges[2];
  vector<double>& line4 = rectEdges[3];

//  double a_tlbl
  line1[0]  = (topLeft[1] - bottomLeft[1]);
//  double a_tltr 
  line2[0] = (topLeft[1] - topRight[1]);
//  double a_blbr 
  line3[0] = (bottomLeft[1] - bottomRight[1]);
//  double a_brtr 
  line4[0] = (bottomRight[1] - topRight[1]);

//  double b_tlbl 
  line1[1] = -(topLeft[0] - bottomLeft[0]);
//  double b_tltr 
  line2[1] = -(topLeft[0] - topRight[0]);
//  double b_blbr 
  line3[1] = -(bottomLeft[0] - bottomRight[0]);
//  double b_brtr 
  line4[1] = -(bottomRight[0] - topRight[0]);

    /*
//  double c_tlbl 
  line1[2] = (topLeft[0] - bottomLeft[0])* topLeft[1] + 
    (-topLeft[1] + bottomLeft[1])* topLeft[0];
//  double c_tltr 
  line2[2] = (topLeft[0] - topRight[0]) * topLeft[1] + 
    (-topLeft[1] + topRight[1]) * topLeft[0];
//  double c_blbr 
  line3[2] = (bottomLeft[0] - bottomRight[0]) * bottomLeft[1] + 
    (-bottomLeft[1] + bottomRight[1]) * bottomLeft[0];
//  double c_brtr 
  line4[2] = (bottomRight[0] - topRight[0]) * bottomRight[1] + 
    (-bottomRight[1] + topRight[1]) * bottomRight[0];
        */

    line1[2] = topLeft[0]*bottomLeft[1] - bottomLeft[0]*topLeft[1];
    line2[2] = topLeft[0]*topRight[1] - topRight[0]*topLeft[1];
    line3[2] = bottomLeft[0]*bottomRight[1] - bottomRight[0]*bottomLeft[1];
    line4[2] = bottomRight[0]*topRight[1] - topRight[0]*bottomRight[1];

}


bool geometry::ComputeRectLineIntersec(double* line1, 
    vector< vector<double> >& rectEdges, double* point1, double* point2) {
  vector< vector<double> > interPts(4);
  vector<int> validInters;
  double height = -rectEdges[0][0]; //Doubtful if this should remain the same
  double width = rectEdges[1][1];

  for(int i=0; i < 4; i++) {
    interPts[i].resize(2,0);  
    bool status = geometry::ComputeLineLineIntersec(line1, 
        rectEdges[i].data(), interPts[i].data());

    if(status) {
      if(interPts[i][0] > width || 
          interPts[i][1] > height || 
          interPts[i][0] < 0 || 
          interPts[i][1] < 0){
          status = false;
        } else {
          validInters.push_back(i);
        }
    }  
  }

  if(validInters.size() == 3) {
//    cout << "\nERROR : 3 intersections";
    return false;
  } else if(validInters.size() <= 1) {
//    cout << "\nERROR: Insufficient intersections";
    return false;
  } else { 
    point1[0] = interPts[validInters[0]][0];
    point1[1] = interPts[validInters[0]][1];

    point2[0] = interPts[validInters[1]][0];
    point2[1] = interPts[validInters[1]][1];
  }
  return true;
}

bool geometry::ComputeLineLineIntersec(double* line1, 
    double* line2, double* pt) {
  double den = -line2[0]*line1[1] + line1[0]*line2[1];
  if(den == 0.0f) {
    return false;
  }
  double num_x = line1[1]*line2[2] - line1[2]*line2[1];
  double num_y = line1[2]*line2[0] - line1[0]*line2[2];

//  printf("\nNumX:%f, NumY:%f, Den:%f", num_x, num_y, den);

  pt[0] = num_x/den;
  pt[1] = num_y/den;

//  printf("\n\nX:%f, Y:%f", pt[0], pt[1]);
  return true; 
}


int geometry::ComputeEpipolarLine( double* x, double* F, 
    double* l, bool fTranspose) {
  if(l == NULL) { 
    return 0;
  }
  double L[3];
  if(fTranspose) {
  //  matrix_transpose_product(3,3,3,1,F,x,l);
   l[0] = F[0]*x[0] + F[3]*x[1] + F[6]*x[2];
    l[1] = F[1]*x[0] + F[4]*x[1] + F[7]*x[2];
    l[2] = F[2]*x[0] + F[5]*x[1] + F[8]*x[2];

  } else {
//    matrix_product(3,3,3,1,F,x,l);
/*
    printf("\nF:");
    for(int ii=0; ii < 9; ii++) {
      printf("%lf ",F[ii]);
    }
*/
//    printf("\nx: %lf %lf %lf", x[0], x[1], x[2]);

    L[0] = F[0]*x[0] + F[1]*x[1] + F[2]*x[2];
    L[1] = F[3]*x[0] + F[4]*x[1] + F[5]*x[2];
    L[2] = F[6]*x[0] + F[7]*x[1] + F[8]*x[2];

//    printf("\nL: %lf %lf %lf", L[0], L[1], L[2]);

    l[0] = F[0]*x[0] + F[1]*x[1] + F[2]*x[2];
    l[1] = F[3]*x[0] + F[4]*x[1] + F[5]*x[2];
    l[2] = F[6]*x[0] + F[7]*x[1] + F[8]*x[2];
//    printf("\nl: %f %f %f", l[0],l[1],l[2]);
  }
  return 1;
}


float geometry::ComputeDistanceFromLine( double* x, double* l) {
  float dist= x[0]*l[0] + x[1]*l[1] + l[2];
  float norm_dist = sqrt(l[0]*l[0] + l[1]*l[1]);
  if(dist < 0) dist = dist*(-1);
  float ans = dist/norm_dist;
  return ans;
}


float geometry::ComputeDistance( double* x1, double* x2, double* F, int verbose) 
{
  double norm_f = 0, Fnorm[12];

  for(int i=0;i<12;i++)  
    norm_f = norm_f + F[i]*F[i];
  norm_f = sqrt(norm_f);  
  for(int i=0;i<12;i++)
    Fnorm[i] = F[i];//*1.0f/norm_f;
  
  double l[3];
    
  double norm_x2[] = {x2[0],x2[1]};
  norm_x2[0] = norm_x2[0]; //(sqrt(x2[0]*x2[0] + x2[1]*x2[1]));
  norm_x2[1] = norm_x2[1];//(sqrt(x2[0]*x2[0] + x2[1]*x2[1]));
  //matrix_product(3,3,3,1,Fnorm,x1,l);
  //
  l[0] = Fnorm[0]*x1[0] + Fnorm[1]*x1[1] + Fnorm[2]*x1[2];
  l[1] = Fnorm[3]*x1[0] + Fnorm[4]*x1[1] + Fnorm[5]*x1[2];
  l[2] = Fnorm[6]*x1[0] + Fnorm[7]*x1[1] + Fnorm[8]*x1[2];
  double norm_l = sqrt(l[0]*l[0] + l[1]*l[1]);// + l[2]*l[2]);
  l[0]=l[0]; //norm_l;
  l[1]=l[1];//norm_l;
  l[2]=l[2];//norm_l;
      
  double dist= (float)(abs(norm_x2[0]*l[0] + norm_x2[1]*l[1] + l[2]));
  double norm_dist = sqrt(l[0]*l[0] + l[1]*l[1]);
  return (float)(dist)/norm_dist;
}

