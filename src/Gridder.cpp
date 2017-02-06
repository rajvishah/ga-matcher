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

#include "Gridder.h"

Gridder::Gridder(int gSize, int imWidth, int imHeight, int numKeys, 
    keypt_t* keyInfo) {
    initialize(gSize, imWidth, imHeight, numKeys, keyInfo);
}

void Gridder::initialize(int gSize, int imWidth, int imHeight, 
    int numKeys, keypt_t* keyInfo) {
    gridSize = gSize;
    imageWidth = imWidth;
    imageHeight = imHeight;
    if(gridSize%2 == 1) gridSize++;
    halfSize = gridSize/2;

    numXGrids1 = floor(imageWidth/gridSize);
    numYGrids1 = floor(imageHeight/gridSize);
    numXGrids2 = floor((imageWidth-halfSize)/gridSize);
    numYGrids2 = floor((imageHeight-halfSize)/gridSize);

    numGrids = numXGrids1*numYGrids1;
    numGridsXOv = numXGrids2*numYGrids1;
    numGridsYOv = numXGrids1*numYGrids2;

    float overlap = (float)halfSize/(float)gridSize;

    for(int i=0; i < numKeys; i++) {
        float x = keyInfo[i].x;
        float y = keyInfo[i].y;

        if(x > (imageWidth - gridSize) ||
                y > (imageHeight - gridSize)) continue;

        if(x < gridSize ||
                y < gridSize) continue;

        vector<int> idx;
        getGridIndices(x,y,idx);
       
        /*
        printf("\nGrids for (%f,%f): ", x, y);
        for(int j=0; j < idx.size(); j++) {
            printf("%d ,", idx[j]);
        }
        */

        vector<int>& bucket1 = gridToPointIndex[idx[0]];
        vector<int>& bucket2 = gridToPointIndex[idx[1]];
        vector<int>& bucket3 = gridToPointIndex[idx[2]];
        vector<int>& bucket4 = gridToPointIndex[idx[3]];

        bucket1.push_back(i);
        bucket2.push_back(i);
        bucket3.push_back(i);
        bucket4.push_back(i);
    }
}


void Gridder:: getGridIndices(float x, float y, vector<int>& idx) {
    double simpleX = (x/(float)gridSize);
    double simpleY = (y/(float)gridSize);


    idx.resize(4);

    double ov = halfSize*1.0f/gridSize*1.0f; 

    idx[0] = floor(simpleY)*numXGrids1 + floor(simpleX);
    idx[1] = floor(simpleY)*numXGrids2 + floor(simpleX - ov);
    idx[2] = floor(simpleY - ov)*numXGrids1 + floor(simpleX);
    idx[3] = floor(simpleY - ov)*numXGrids2 + floor(simpleX - ov);

    idx[1] = idx[1] + numGrids;
    idx[2] = idx[2] + numGrids + numGridsXOv;
    idx[3] = idx[3] + numGrids + numGridsXOv + numGridsYOv;

}

void Gridder:: getGridIndDists(float x, float y, vector<int>& idx, 
    vector<float>& dists) {

    getGridIndices(x,y,idx);
    double simpleX = (x/gridSize);
    double simpleY = (y/gridSize);
    double ov = halfSize*1.0f/gridSize*1.0f;

    float g1Xc = (floor(simpleX))*gridSize + halfSize;
    float g1Yc = (floor(simpleY))*gridSize + halfSize;

    float g2Xc = (floor(simpleX - ov))*gridSize + 2*halfSize;
    float g2Yc = g1Yc;

    float g3Xc = g1Xc;
    float g3Yc = (floor(simpleY - ov))*gridSize + 2*halfSize;

    float g4Xc = g2Xc;
    float g4Yc = g3Yc;

    float x1_d = (g1Xc - x)*(g1Xc - x);
    float y1_d = (g1Yc - y)*(g1Yc - y);

    float x2_d = (g2Xc - x)*(g2Xc - x);
    float y2_d = (g3Yc - y)*(g3Yc - y);

    dists.resize(4, 10000);
    dists[0] = x1_d + y1_d;
    dists[1] = x2_d + y1_d;
    dists[2] = x1_d + y2_d;
    dists[3] = x2_d + y2_d;
}

int Gridder::getClosestGrid(float x, float y) {
    vector<int> idx(4);
    vector<float> dists(4);
    getGridIndDists(x,y,idx,dists);

    float min_g_dist = 200000;
    int minIdx = -1;
    int chosen_id = -1;
    for(int id=0; id < 4; id++) {
        if(dists[id] < min_g_dist) {
            min_g_dist = dists[id];
            minIdx = idx[id];
            chosen_id = id;
        }
    }
    fflush(stdout);
    return minIdx;
}

void Gridder::getGridPoints(float x, float y, map<int, int>& gridPts) {
    int idx = getClosestGrid(x, y);

    //cout << "Grid Index " << idx << endl;

    vector<int>& pts = gridToPointIndex[idx];
    for(int j=0; j < pts.size(); j++) {
        gridPts.insert(pair< int, int >(pts[j],1.0));
    }
    //cout <<  gridPts.size() << " points added  ";
}


void Gridder::getNearbyGridPoints(float x, float y, vector<int>& gridPts) {
    vector<int> idx(4);
    vector<float> dists(4);
    getGridIndDists(x,y,idx,dists);

    for(int i=0; i < 4; i++) {
        vector<int>& pts = gridToPointIndex[idx[i]];
        gridPts.insert(gridPts.end(), gridPts.begin(), gridPts.end());

        printf("\nGrid %d %d", i, idx[i]); 

        for(int j=0; j < pts.size(); j++) {
            printf("%d ,", pts[j]);
        }
    }
}

void Gridder::getNearbyGridPoints(vector<float> x, vector<float> y, 
    vector<int>& gridPts) {
  for(int i=0; i < x.size(); i++) {
    vector<int> idx(4);
    vector<float> dists(4);
    getGridIndDists(x[i],y[i],idx,dists);
    for(int j=0; j < 4; j++) {
      vector<int>& pts = gridToPointIndex[idx[j]];
      gridPts.insert(gridPts.end(), pts.begin(), pts.end());
    }
  }
}

void Gridder::getGridPoints(vector<float>& x, vector<float>& y, 
    vector<int>& gridPts) {
    map<int,int> gridSet;
    for(int i=0; i < x.size(); i++) {
        getGridPoints(x[i], y[i], gridSet);
    }

    map<int,int>::iterator gset_it = gridSet.begin();
    gridPts.resize( gridSet.size() );
    for(int i=0; i < gridSet.size(); i++) {
        gridPts[i] = (*gset_it).first;
        gset_it++;
    }
}

