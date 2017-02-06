#ifndef __DEFS_H
#define __DEFS_H

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

#ifdef DEBUG 
#define DBG(x) x
#else 
#define DBG(x)
#endif

#include <string.h>
#include <stdlib.h>
#include <cmath>

#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>

#include <fstream>
#include <iostream>
#include <jpeglib.h>
using namespace std;
 
#endif //__DEFS_H 
