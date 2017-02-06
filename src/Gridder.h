#ifndef __GRIDDER_H
#define __GRIDDER_H 

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

class Gridder {
	int gridSize;
	int halfSize;
	int imageWidth;
	int imageHeight;

	int numXGrids1;
	int numYGrids1;
	int numXGrids2;
	int numYGrids2;

	int numGrids;
	int numGridsXOv;
	int numGridsYOv;

	map<int, vector<int> > gridToPointIndex;

	int getClosestGrid(float x, float y);

	void getGridIndices(float x, float y, vector<int>& idx);
	void getGridIndDists(float x, float y, vector<int>& idx, vector<float>& dists);
public:
	Gridder(){}
	Gridder(int gSize, int imWidth, int imHeight, int numKeys, keypt_t* keys);
	void initialize(int gSize, int imWidth, int imHeight, int numKeys, keypt_t* keys);
	void getGridPoints(float x, float y, map<int, int>& gridPts);
	void getNearbyGridPoints(float x, float y, vector<int>& gridPts);
  void getNearbyGridPoints(vector<float> x, vector<float> y, vector<int>& gridPts);
	void getGridPoints(vector<float>& x, vector<float>& y, vector<int>& gridPts);
};



#endif //__GRIDDER_H 
