#ifndef __MATCHER_H
#define __MATCHER_H 
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
#include "keys2a.h"
#include "Gridder.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace match {

class FeatureMatcher{
  int numSrcPts;
	unsigned char* srcKey;
	keypt_t* srcKeysInfo;

  int numRefPts;
	keypt_t* refKeysInfo;
	unsigned char* refKey;

    int qWidth, qHeight, rWidth, rHeight;

	vector< double > fMatrix;
	vector< vector < double > > srcRectEdges;
	vector< vector < double > > refRectEdges;

    vector< vector<double> > epiLines;
    vector< vector<double> > lineEndPointGroups;
    vector< int > groupEpiLineIdx;
    vector< int > pointToLineGroupIdx;

    vector <vector<int> > pointGroups;

    //vector<pair<int, int> > matches;
    vector< ANNkd_tree* > refImgKdTrees;

    Gridder* qGrid;
    Gridder* rGrid;

    public:

    cv::Mat queryImage;
    cv::Mat referenceImage;
    void verifyEpipolarConstraints(); 

    void validateProbableMatches();
    void visualizeMatches(const char*);
    vector<pair<int, int> > matches;
    void setQueryGrid(Gridder* grid) {
        qGrid = grid;
    }
    void setRefGrid(Gridder* grid) {
        rGrid = grid;
    }

    void setNumSrcPoints(int nSrcPts) {
        numSrcPts = nSrcPts;
    }

    void setNumRefPoints(int nRefPts) {
        numRefPts = nRefPts;
    }

    void setFMatrix(vector<double>& f) {
        if(f.empty()) {
          f.resize(9);
        }
        fMatrix = f;
    }

    void setSrcKeys(keypt_t* info, unsigned char* key) {
        srcKeysInfo = info;
        srcKey = key;
    }

    void setRefKeys(keypt_t* info, unsigned char* key) {
        refKeysInfo = info;
        refKey = key;
    }


    void setImageDims(int qW, int qH, int rW, int rH) {
        qWidth = qW;
        qHeight = qH;
        rWidth = rW;
        rHeight = rH;
    }
    void setSrcRectEdges(vector< vector< double > >& rectEdges) {
        srcRectEdges = rectEdges;
    }


    void setRefRectEdges(vector< vector< double > >& rectEdges) {
        refRectEdges = rectEdges;
    }

    double* getFMatrixPtr() {
        return fMatrix.data();
    }

    
    bool image2norm(keypt_t* in, double* out, bool source) {
      double width = source ? qWidth : rWidth;
      double height = source ? qHeight : rHeight; 
      out[0] = in->x - (width/2);
      out[1] = (height/2) - in->y;
      out[2] = 1.0f; 
    }

    bool image2norm(double* in, double* out, bool source) {
      double width = source ? qWidth : rWidth;
      double height = source ? qHeight : rHeight; 
      out[0] = in[0] - (width/2);
      out[1] = (height/2) - in[1];
      out[2] = in[2];  
    }
    
    bool norm2image(double* in, double* out, bool source) {
      double width = source ? qWidth : rWidth;
      double height = source ? qHeight : rHeight; 
      out[0] = in[0] + (width/2);
      out[1] = (height/2) - in[1];
      out[2] = in[2];   
    } 
    
    bool norm2image(double* in, keypt_t* out, bool source) { 
      double width = source ? qWidth : rWidth;
      double height = source ? qHeight : rHeight; 
      out->x = (in[0] - (width/2))/in[2];
      out->y = ((height/2) - in[1])/in[2];
    }
    
    /* pass non-negative x,y coordinates of line endpoints */
    unsigned long long int packLineEndPoints(short int x1, 
        short int y1, short int x2, short int y2) { 
      unsigned long long int l = 0;
      l = l | ((unsigned long long int)x1 << 48);
      l = l | ((unsigned long long int)y1 << 32);
      l = l | ((unsigned long long int)x2 << 16);
      l = l | ((unsigned long long int)y2 << 0);
      return l; 
    }

    void unpackLineEndPoints(unsigned long long int l,
        short int* x1, short int* y1, short int* x2, short int* y2) {
      unsigned long long int s = 0;
      s = ~s;
      s = s >> 48;
      *x1 = (short)(s & (l >> 48));
      *y1 = (short)(s & (l >> 32));
      *x2 = (short)(s & (l >> 16));
      *y2 = (short)(s & (l));
    }


    int visualizeClusters();
    int verifyFeatureMatches(double* F);
    int computeFmatrix(double* Fdata);
    void computeEpipolarLines();
    void clusterPoints();
    void clusterPointsFast();
    int match();
    int bfMatch();
    int globalMatch(int h, bool twoway);
    ANNkd_tree* constructSearchTree(int idx, vector<int>& probMatches);
    void getProbableMatches(int idx, vector<int>& probMatches);
};

};
#endif //__MATCHER_H 
