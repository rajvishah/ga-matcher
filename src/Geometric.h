#ifndef __GEOMETRIC_H
#define __GEOMETRIC_H

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

#include <vector>

using namespace std;
namespace geometry {


   
void ComputeRectangleEdges(double width, double height, vector < vector<double> >& rectEdges);
bool ComputeRectLineIntersec(double* line1, vector< vector<double> >& rectEdges, double* point1, double* point2);

bool ComputeLineLineIntersec(double* line1, double* line2, double* pt);
void ComputeFundamental(double* P1, double* P2, double* C, double* F);
int ComputeEpipolarLine( double* x, double* F, double* l, bool fTranspose = false);
float ComputeDistanceFromLine( double* x, double* l);
float ComputeDistance( double* x1, double* x2, double* F, int verbose);
};
#endif //__GEOMETRIC_H 
