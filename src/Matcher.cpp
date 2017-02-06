#include "Matcher.h"
#include "defs.h"
#include "keys2a.h"
#include "Geometric.h"

/*
 * Author : Rajvi Shah (rajvi.a.shah@gmail.com)
 * SIFT-like feature matching implementation introduced in the following paper:
 *
 * "Geometry-aware Feature Matching for Structure from Motion Applications", 
 * Rajvi Shah, Vanshika Srivastava and P J Narayanan, WACV 2015.
 * http://researchweb.iiit.ac.in/~rajvi.shah/projects/multistagesfm/
 *
 *
 * Copyright (c) 2015 Rajvi Shah with International Institute of Information 
 * Technology - Hyderabad 
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


using namespace match;

/*! \brief Computes Fundamental Matrix for initialized matches.
 **
 **  This is a wrapper function that prepares (x,y) pairs of matches 
 **  and calls the OpenCV function findFundamentalMat(...). 
 **/
int FeatureMatcher::computeFmatrix(double* Fdata) {
  vector<cv::Point2f> imgPts1, imgPts2;
  for(int i=0; i < matches.size(); i++) {
    int idx1 = matches[i].first;
    int idx2 = matches[i].second;

    imgPts1.push_back( cv::Point2f(srcKeysInfo[idx1].x,srcKeysInfo[idx1].y));
    imgPts2.push_back( cv::Point2f(refKeysInfo[idx2].x,refKeysInfo[idx2].y));

  }
  cv::Mat Finliers;

  /// Please see OpenCV documentation before you change parameters
  cv::Mat F = findFundamentalMat(imgPts1, imgPts2, CV_FM_RANSAC, 1, 0.99, Finliers);

  /// Delete all outlier matches
  vector<pair<int, int> > :: iterator itr = matches.begin();
  for (int i=0; i< Finliers.size().height; i++) {
    if (Finliers.at<char>(i) != 1) {
      matches.erase(itr);  
    }
    itr++;
  }

  /// Check if sufficient matches remian inliers
  //  This can be made stricter (See paper)
  int matchCount = (int)(matches.size());
  if(matchCount >= 27) {
    const double* ptr = F.ptr<double>();
    for(int f=0; f < 9; f++) {
      Fdata[f] = ptr[f]/ptr[8];
    }

    return (int)matches.size();
  }

  return -1;
}


/*! \brief Computes epipolar lines for all source feature points.
 **        The computed lines are stored in vector< double > epiLines 
 **/
void FeatureMatcher::computeEpipolarLines() {
  epiLines.resize(numSrcPts);
  for(int i=0; i < numSrcPts; i++) {
    epiLines[i].resize(3);
    double x1[] = {srcKeysInfo[i].x, srcKeysInfo[i].y, 1.0};
    geometry::ComputeEpipolarLine(x1, fMatrix.data(), 
        epiLines[i].data(), false);
  }
}

/*! \brief Removes matches that do not satisfy epipolar constraints.
 **
 **  This is a useful function to verify matches found in  the second stage. 
 **/
void FeatureMatcher::verifyEpipolarConstraints() {

  ///For all matching pairs in the matches vector
  for(int f=0; f < matches.size(); f++) {
    int i = matches[f].first;           // Source Index
    int matchingPt = matches[f].second; // Reference Index

    /// Compute epipolar line for the source point
    double x1[] = {srcKeysInfo[i].x, srcKeysInfo[i].y, 1.0};
    double *line1 = new double[3];
    double* F = fMatrix.data();
    geometry::ComputeEpipolarLine(x1, F, line1, false);

    double x2[] = {refKeysInfo[matchingPt].x, refKeysInfo[matchingPt].y, 1.0};

    /// Compute distance of the reference point from epipolar line
    float epiDist = geometry::ComputeDistanceFromLine( x2, line1);

    /// If distance is above a threshold, delete the match
    if(epiDist >= 4.0) {
      matches.erase(matches.begin()+f);
    }
  }

}


/*! \brief This function computes matches using global Kd-tree based method.
 **
 **  This function is useful in two ways. 
 **  (1) To compute initial matches using eta% features during first stage of matching. 
 **  (2) To compare performance of geometry based method using 100% features.
 **  
 **  Parameters: 
 **  int h - Percentage of total features to use for matching
 **          This would be 10,20 for initial matching, 100 for full matching 
 **  bool twoWaySearch - whether the match satisfies two-way matching constraint.
 **                      if R in I2 is a matching point for S in I1, then S 
 **                      should be the matching point in I1 for R in I2.
 **  
 **/

int FeatureMatcher::globalMatch(int h, bool twoWaySearch) {
  /// Clear previously computed matches if any
  matches.clear();

  /// Find number of h% matches
  int numTopRefPts = (int)(numRefPts*h/100);  
  int numTopSrcPts = (int)(numSrcPts*h/100);  

  /// Allocate point structures for Kd-tree based search
  ANNpointArray keyPts = annAllocPts( numTopRefPts, 128);
  ANNpointArray qKeyPts = annAllocPts( numTopSrcPts, 128);

  /// Copy descriptors to allocated point structures
  for(int i=0; i < numTopRefPts; i++) 
    memcpy(keyPts[i], refKey+128*i, sizeof(unsigned char)*128);

  for(int i=0; i < numTopSrcPts; i++) 
    memcpy(qKeyPts[i], srcKey+128*i, sizeof(unsigned char)*128);

  /// Create trees for source and target descriptors
  ANNkd_tree* tree = new ANNkd_tree(keyPts, numTopRefPts, 128, 16);
  ANNkd_tree* qTree = new ANNkd_tree(qKeyPts, numTopSrcPts, 128, 16);

  /// Number of nodes to visit in Kd-tree (standard practice)
  /// Limit this number to the lesser 500 or TotalPoints/20
  /// If the 20% points are too less, limit it at least to 50
  int PtsToVisit = numTopRefPts > numTopSrcPts ? numTopRefPts : numTopSrcPts;

  PtsToVisit = ceil((float)PtsToVisit/(float)20) < 
    500 ? ceil((float)PtsToVisit/(float)20) : 500;

  if(PtsToVisit < 50) PtsToVisit = 50;

  annMaxPtsVisit(PtsToVisit);
  //printf("\nPts To Visit : %d", PtsToVisit);

  /// For each of the selected source features
  for(int i=0; i < numTopSrcPts; i++) {

    vector<ANNidx> indices(2);
    vector<ANNdist> dists(2);

    /// Search for two closest points in the reference tree
    unsigned char* qKey = srcKey + 128*i;
    tree->annkPriSearch(qKey, 2, indices.data(), dists.data(), 0.0);

    /// Compute best distance to second best distance ratio
    float bestDist = (float)(dists[0]);
    float secondBestDist = (float)(dists[1]);

    float distRatio = sqrt(bestDist/secondBestDist);

 //   printf("\nSrc Point %d, best dist %f, second best %f, ratio %f, 1st %d, 2nd %d",
 //       i, bestDist, secondBestDist, distRatio, (int)indices[0], (int)indices[1]);

 
    /// If the ratio is larger than threshold, match is not considered
    if(distRatio > 0.6) {
      continue;
    }

    /// If the ratio is below the threshold, the closest point is the match
    int matchingPt = (int)indices[0];
    int secondMatch = (int)indices[1];
    indices.clear();
    dists.clear();

    /// If two way search is enabled, verify that 
    //  the query point is the best match for the
    //  matching point and also satisfies ratio test
    if(twoWaySearch) {

      unsigned char* qKey1 = refKey + 128*matchingPt;
      qTree->annkPriSearch(qKey1, 2, indices.data(), dists.data(), 0.0);

      float bestDist1 = (float)(dists[0]);
      float secondBestDist1 = (float)(dists[1]);

      float distRatio1 = sqrt(bestDist1/secondBestDist1);

      if((int)(indices[0]) != i) {
        continue;
      }

      if(distRatio1 > 0.6) {
        continue;
      }
    }

//    printf("\nSrc Point %d, 1st %d, 2nd %d",
//        i, matchingPt, secondMatch);
    /// Add the pairs to matches list
    matches.push_back(make_pair(i, matchingPt)); 
  }

  /// Deallocate Point Structures
  annDeallocPts(keyPts);
  annDeallocPts(qKeyPts);

  /// Delete Kd-tree
  delete tree;
  delete qTree;
  return (int)matches.size();
}

/*! \brief For visualizing matches side-by-side and saving image results.
 **
 **  Parameter : Path to save image (optional) in OpenCV accepted format. 
 **/
void FeatureMatcher::visualizeMatches(const char* saveAs = NULL) {
  cv::Size sz = queryImage.size();
  int qWidth = sz.width;
  int qHeight = sz.height;

  cv::Size sz1 = referenceImage.size();
  int rWidth = sz1.width;
  int rHeight = sz1.height;

  int canvasWidth = qWidth + rWidth;
  int canvasHeight = qHeight > rHeight ? qHeight : rHeight;
  cv::Mat canvas(canvasHeight, canvasWidth, CV_8UC3, cv::Scalar(0,0,0));

  cv::Rect roi1 = cv::Rect(0,0,qWidth,qHeight);
  cv::Mat canvas_roi1 = canvas(roi1);
  queryImage.copyTo(canvas_roi1);

  cv::Rect roi2 = cv::Rect(qWidth,0,rWidth,rHeight);
  cv::Mat canvas_roi2 = canvas(roi2);
  referenceImage.copyTo(canvas_roi2);

  for(int i=0; i < matches.size(); i++) {
    pair<int,int> p = matches[i];
    cv::Point pt1 = cv::Point(srcKeysInfo[p.first].x, srcKeysInfo[p.first].y);
    cv::Point pt2 = cv::Point(refKeysInfo[p.second].x + qWidth, refKeysInfo[p.second].y);

    cv::circle(canvas, pt1, 2, cv::Scalar(0,255,0), 4);
    cv::circle(canvas, pt2, 2, cv::Scalar(0,255,0), 4);
    cv::line(canvas, pt1, pt2, cv::Scalar(0,255,0), 4);
  }

  cv::namedWindow("FeatureMatches",cv::WINDOW_NORMAL);
  imshow("FeatureMatches", canvas);
  cv::waitKey();
  if(saveAs != NULL) {
    imwrite( saveAs, canvas );
  }
}

/*! \brief Matching with brute-force distance computation in the candidate set.
 **
 **  Geometry-aware approach limits the search of a match to a small set of
 **  points around the epiploar line (mentioned as candidate set in paper,      
 **  vector<int> probMatches in the code). The default function builds a 
 **  small Kd-tree of these candidate features to find the top-two candidates.
 **  This function uses brute-force (exhaustive) distance computation for it.
 **  It is perhaps less efficient in time but more accurate. 
 **/
int FeatureMatcher:: bfMatch() {
  matches.clear();
  /// For all groups of points clustered based on their epipolar lines
  /// Read clusterPointsFast() to see implementation details
  for(int i=0; i < pointGroups.size(); i++) {   
    int qPtSize = pointGroups[i].size();

    /// Get corresponding epipolar line for this group of pts
    int idx = groupEpiLineIdx[i];
    int groupIdx = pointToLineGroupIdx[idx];

    /// Get features close to the epipolar line
    vector<int> probMatches;
    getProbableMatches(idx, probMatches);

    if(probMatches.size() == 0) {
      continue;
    }

    ///  For each points within a cluster, find descriptor space distance 
    ///  from all the points in the candidate set (probMatches) 
    ///  Inster the points in a map, to sort in order of distance
    for(int j=0; j < pointGroups[i].size(); j++) {

      int qPtIdx = pointGroups[i][j];

      unsigned char* currQuery = srcKey + 128*qPtIdx;
      multimap<float,int> distMap;

      for(int jj=0; jj < probMatches.size(); jj++) {
        unsigned char* refVector = refKey + 128*probMatches[jj];

        float acc = 0.0;
        for(int dd=0; dd < 128; dd++) {
          acc = acc + (refVector[dd] - currQuery[dd])*
            (refVector[dd] - currQuery[dd]);
        }
        distMap.insert(make_pair(acc,probMatches[jj])); 
      }

      multimap<float,int>::iterator itr;
      itr = distMap.begin();

      pair<float,int> bestMatch = *itr;
      itr++;
      pair<float,int> secondBestMatch = *(itr);

      /// Perform ratio-test between closest two points
      /// Discard the match if ratio is above a threshold
      float bestDist = (float)bestMatch.first;
      float secondBestDist = (float)secondBestMatch.first;
      float distRatio1 = sqrt(bestDist/secondBestDist);

      distMap.clear();

      if(distRatio1 > 0.6) {
        continue;
      }

      int matchingPt = bestMatch.second;

      /// Perform epipolar verification
      double x2[] = {refKeysInfo[matchingPt].x, 
        refKeysInfo[matchingPt].y, 1.0};
      double line[3];
      geometry::ComputeEpipolarLine(x2,fMatrix.data(),line,true); 

      double x1[] = {srcKeysInfo[qPtIdx].x,
        srcKeysInfo[qPtIdx].y, 1.0};
      float epiDist2 = 
        geometry::ComputeDistanceFromLine( x1, line);
      if(epiDist2 >= 4.0) {
        continue;
      }
      matches.push_back(make_pair(qPtIdx, matchingPt));
    }
  }
  int matchCount = (int)(matches.size());
  return matchCount;
}

/*! \brief Default Matching (core function) of geometry-aware approach.
 **
 **  Geometry-aware approach limits the search of a match to a small set of
 **  points around the epiploar line (mentioned as candidate set in paper,      
 **  vector<int> probMatches in the code). The default function builds a 
 **  small Kd-tree of these candidate features to find the top-two candidates.
 **  A brute-force search in candidate set is also implemented. See bfMatch().
 **/

int FeatureMatcher::match() {
  matches.clear();
  /// For all groups of points clustered based on their epipolar lines
  /// Read clusterPointsFast() to see implementation details

  for(int i=0; i < pointGroups.size(); i++) {  
    int qPtSize = pointGroups[i].size();

    /// Get corresponding epipolar line for this group of pts
    int idx = groupEpiLineIdx[i];
    int groupIdx = pointToLineGroupIdx[idx];

    /// Get features close to the epipolar line and construct a Kd-tree 
    /// of its descriptors

    vector<int> probMatches;
    ANNkd_tree* tree = constructSearchTree(idx, probMatches);

    /// Limit the nodes to visit in this tree as max(20% of candidates,20)
    int PtsToVisit = (float)(probMatches.size())/20;
    PtsToVisit = PtsToVisit > 20 ? PtsToVisit : 20;
    annMaxPtsVisit(PtsToVisit);

    if(tree == NULL) {
      continue;
    }

    /// For each points within a cluster, find the closest two points
    /// from the candidate set (probMatches) using Kd-tree in descriptor
    /// space and perform ratio-test
    for(int j=0; j < pointGroups[i].size(); j++) {
      vector<ANNidx> nn_idx(2);
      vector<ANNdist> dists(2);

      vector<ANNidx> nn_idx1(2);
      vector<ANNdist> dists1(2);

      int qPtIdx = pointGroups[i][j];
      unsigned char* currQuery = srcKey + 128*qPtIdx;
      tree->annkPriSearch(currQuery, 2, nn_idx.data(), dists.data(), 0.0);


      /// Perform ratio-test between closest two points
      /// Discard the match if ratio is above a threshold
      float bestDist = (float)(dists[0]);
      float secondBestDist = (float)(dists[1]);
      float distRatio1 = sqrt(bestDist/secondBestDist);

      if(distRatio1 > 0.6) {
        continue;
      }

      int matchingPt = probMatches[(int)nn_idx[0]];

      /// Perform epipolar verification
      double x2[] = {refKeysInfo[matchingPt].x,
        refKeysInfo[matchingPt].y, 1.0};
      double line[3];
      geometry::ComputeEpipolarLine(x2,fMatrix.data(),line,true); 

      double x1[] = {srcKeysInfo[qPtIdx].x,
        srcKeysInfo[qPtIdx].y, 1.0};
      float epiDist2 = 
        geometry::ComputeDistanceFromLine( x1, line);
      if(epiDist2 >= 4.0) {
        continue;
      }
      matches.push_back(make_pair(qPtIdx, matchingPt));

    }

    /// Free memory.
    annDeallocPts(tree->pts);
    delete tree;
  }

  int matchCount = (int)(matches.size());
  return matchCount;
}

/*! \brief Finds the candidate set using grid-based search
 **
 **  For a given point, finds features within 4px of its epipolar line
 **  Uses the grid-based search method explained in the WACV 2015 paper 
 **/
void FeatureMatcher::getProbableMatches(int idx, vector<int>& probMatches) {
  double* currLine = epiLines[idx].data();
  int groupIdx = pointToLineGroupIdx[idx];
  vector<double>& endPoints = lineEndPointGroups[groupIdx];
  float lineWidth = endPoints[0] - endPoints[2];
  float lineHeight = endPoints[1] - endPoints[3];
  float lineLength = sqrt(lineWidth*lineWidth + lineHeight*lineHeight);

  int numLinePts = (int)floor(lineLength/4);

  vector<float> x(numLinePts);
  vector<float> y(numLinePts);

  for(int i=0; i < numLinePts; i++) {
    float k1 = i+1;
    float k2 = numLinePts - k1;
    x[i] = (k1*endPoints[2] + k2*endPoints[0])/(k1 + k2);
    y[i] = (k1*endPoints[3] + k2*endPoints[1])/(k1 + k2);
  }

  rGrid->getGridPoints(x, y, probMatches);
   // If probable matches are too few, ratio-test is meaning less and
    // can generate false positives and add noise
    // Add a random set of points as candidates in this case
    if(probMatches.size() > 0 && probMatches.size() < 50) {
        set<int> randIndices;
        set<int>::iterator setItr;
        pair<set<int>::iterator,bool> ret;

        while(probMatches.size() < 50) {
            int rand_index = rand() % numRefPts;
            ret = randIndices.insert( rand_index );
            if(ret.second == true) {
                probMatches.push_back(rand_index);
            }
        }
    }
}


/*! \brief Finds the candidate set and builds a Kd-tree of its descriptors.
 **/
ANNkd_tree* FeatureMatcher::constructSearchTree(int idx, vector<int>& probMatches) {
  getProbableMatches(idx, probMatches);
  if(probMatches.size() == 0) {
    return NULL;
  }

  int numSubKeys = probMatches.size();
  ANNpointArray subKeyPts = annAllocPts( numSubKeys, 128 );
  for(int p=0; p < numSubKeys; p++) {
    int pm = probMatches[p];
    memcpy(subKeyPts[p], refKey + 128*pm,
        sizeof(unsigned char)*128);
  }

//  printf("\nProb Matches size = %d", probMatches.size());

  ANNkd_tree* subTree = new ANNkd_tree(subKeyPts, numSubKeys, 128, 16);
  return subTree;
}

/*! \brief Clusters feature points based on their epipolar lines (Fast)
 **
 **  This function sorts all epipolar lines based on their endpoints
 **  And groups the lines within 4 pixel distance in x or y
 **  Endpoints are packed into a long long int for faster computation
 **/
void FeatureMatcher::clusterPointsFast() {

  lineEndPointGroups.reserve(2000);
  pointGroups.reserve(2000);

  pointToLineGroupIdx.resize(numSrcPts);
  vector< pair<unsigned long long int, int> > clubSortedArr( epiLines.size() );

  for(int i=0; i < epiLines.size(); i++) {
    double endPoint1[2], endPoint2[2];
    double* currEpiLine = epiLines[i].data();
    bool status = geometry::ComputeRectLineIntersec(currEpiLine,
        refRectEdges, endPoint1, endPoint2);
    if(!status) {
      continue;
    }

    short int x1 =0, x2 = 0, y1 = 0, y2 = 0;
    x1 = (short)(endPoint1[0]); 
    y1 = (short)(endPoint1[1]);
    x2 = (short)(endPoint2[0]);
    y2 = (short)(endPoint2[1]);

    unsigned long long int l = packLineEndPoints(x1,y1,x2,y2);
    
    clubSortedArr[i].first = l;
    clubSortedArr[i].second = i;

  }

  sort(clubSortedArr.begin(), clubSortedArr.end());
  vector<double> newEndPoints(4,0);

  int pCount = -1;
  float dist1 = 100, dist2 = 100;
  for(int i= 0; i < epiLines.size(); i++) {
  
    int idx = clubSortedArr[i].second;
    
    unsigned long long int l = clubSortedArr[i].first;
    short int x1 =0, x2 = 0, y1 = 0, y2 = 0;
    unpackLineEndPoints(l,&x1,&y1,&x2,&y2);

    if(i > 0) {
      dist1 = abs(newEndPoints[0] - x1) + abs(newEndPoints[1] - y1);
      dist2 = abs(newEndPoints[2] - x2) + abs(newEndPoints[3] - y2);
    }

    if(dist1 < 4 && dist2 < 4) {
      pointGroups[pCount].push_back(idx);
      pointToLineGroupIdx[idx] = pCount;
    } else {
      newEndPoints[0] = (double)x1; 
      newEndPoints[1] = (double)y1; 
      newEndPoints[2] = (double)x2; 
      newEndPoints[3] = (double)y2;

      lineEndPointGroups.push_back(newEndPoints);
      groupEpiLineIdx.push_back(idx);
      vector<int> initVec;
      pointGroups.push_back(initVec);
      pCount += 1;
      pointGroups[pCount].push_back(idx);
      pointToLineGroupIdx[idx] = pCount;
    }
  }
  int numGroups = pointGroups.size();
}


/*! \brief Clusters feature points based on their epipolar lines (Slower)
 **
 **  This is a map based approach (roughly n^2 complexity)
 **  This is slower than clusterPointsFast() but better tested
 **/
void FeatureMatcher::clusterPoints() {
  lineEndPointGroups.reserve(2000);
  pointGroups.reserve(2000);
  pointToLineGroupIdx.resize(numSrcPts);
  for(int i=0; i < epiLines.size(); i++) {
    double endPoint1[2], endPoint2[2];
    double* currEpiLine = epiLines[i].data();
    bool status = geometry::ComputeRectLineIntersec(currEpiLine,refRectEdges, endPoint1, endPoint2);
    if(!status) {
      continue;
    }

    endPoint1[0] = round(endPoint1[0]);
    endPoint1[1] = round(endPoint1[1]);
    endPoint2[0] = round(endPoint2[0]);
    endPoint2[1] = round(endPoint2[1]);

    bool pointAdded = false;
    for(int j=0; j < lineEndPointGroups.size(); j++) {
      vector<double> refEndPoints = lineEndPointGroups[j]; 

      double refE1_x = round(refEndPoints[0]);
      double refE1_y = round(refEndPoints[1]);
      double refE2_x = round(refEndPoints[2]);
      double refE2_y = round(refEndPoints[3]);

      double dist1 = abs(endPoint1[0] - refE1_x) + abs(endPoint1[1] - refE1_y);
      double dist2 = abs(endPoint2[0] - refE2_x) + abs(endPoint2[1] - refE2_y);

      if(dist1 < 4 && dist2 < 4) {
        pointGroups[j].push_back(i);
        pointToLineGroupIdx[i] = j;
        pointAdded = true;
        break;
      }
    }

    if(!pointAdded) {
      vector<double> newEndPoints(4);
      newEndPoints[0] = endPoint1[0]; //ep1.x
      newEndPoints[1] = endPoint1[1]; //ep1.y
      newEndPoints[2] = endPoint2[0]; //ep2.x
      newEndPoints[3] = endPoint2[1]; //ep2.y
      lineEndPointGroups.push_back(newEndPoints);
      groupEpiLineIdx.push_back(i);
      vector<int> initVec;
      pointGroups.push_back(initVec);
      int index = lineEndPointGroups.size()-1;
      pointGroups[index].push_back(i);
      pointToLineGroupIdx[i] = index;
      pointAdded = true;
    }
  }

  int numGroups = pointGroups.size();
}


/*************************************/
/* Helper code for debug purposes
 * **********************************
 int FeatureMatcher::visualizeClusters() {
 for(int i=0; i < pointGroups.size(); i++) {  
 cv::Mat refClone(cv::Size(rWidth, rHeight), CV_8UC3, cv::Scalar(0, 0, 0));
 referenceImage.copyTo(refClone);
 int qPtSize = pointGroups[i].size();

//get corresponding epipolar line for this group of pts
int idx = groupEpiLineIdx[i];
int groupIdx = pointToLineGroupIdx[idx];
vector<double> epts = lineEndPointGroups[groupIdx];
printf("\nGroup %d", i);
printf("\n");
int counter = 0;
for(int j=0; j < pointGroups[i].size(); j++) {
double endPoint1[2], endPoint2[2];
int qPtIdx = pointGroups[i][j];
double* currEpiLine = epiLines[qPtIdx].data();
bool status = geometry::ComputeRectLineIntersec(currEpiLine,refRectEdges, endPoint1, endPoint2);
if(!status) {
continue;
}
counter++;
int r=0,g=0,b=0;
if(j%1 == 0) {
r = j*10;
} else if(j %2 == 0) {
g = j*10;
} else {
b = j*10;
}

printf("\nEndpoints: %f, %f, %f, %f", endPoint1[0], endPoint1[1], endPoint2[0], endPoint2[1]);

cv::Point pt1(endPoint1[1], endPoint1[0]);
cv::Point pt2(endPoint2[1], endPoint2[0]);
cv::line(refClone, pt1, pt2, cv::Scalar(r,g,b), 4);     
}

printf("\nDrew Lines: %d", counter);
printf("\nEndpoints: %f, %f, %f, %f", epts[0], epts[1], epts[2], epts[3]);
cv::namedWindow("Lines",CV_WINDOW_NORMAL);
cv::imshow("Lines",refClone);
cv::waitKey();
cv::destroyWindow("Lines");
refClone.release();
}
}*/

/*************************************************
 * TO DO
 * **********************************************
 void FeatureMatcher::validateProbableMatches() {
//For all source points
// Find Epi Line
// Find distance of all ref points from epi line
// Find list of points with dist <=4
// Use getProbableMatches to get gridder output
} */



