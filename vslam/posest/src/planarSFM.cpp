/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <cv.h>
#include <iostream>
#include <stdio.h>
#include <limits>

#include "posest/planarSFM.h"
#include "sba/sba.h"

using namespace cv;

//#define DEBUG_CONSOLE

namespace pe
{

struct HomographyDecomposition
{
  HomographyDecomposition()
  {
    Rp = Mat::eye(3, 3, CV_32F);
    Tp = Mat(3, 1, CV_32F);
    Np = Mat(3, 1, CV_32F);
  };

  HomographyDecomposition(const HomographyDecomposition& h)
  {
    h.Rp.copyTo(Rp);
    h.Tp.copyTo(Tp);
    h.Np.copyTo(Np);
    h.R.copyTo(R);
    h.T.copyTo(T);
    dp = h.dp;
    score = h.score;
  };

  Mat Rp;
  Mat Tp;
  Mat Np;
  double dp;

  Mat R;
  Mat T;

  int score;
};

void dumpFltMat(const char* name, const Mat& mat)
{
  printf("Dumping %s\n", name);
  for(int i = 0; i < mat.rows; i++)
  {
    for(int j = 0; j < mat.cols; j++)
    {
      printf("%f ", mat.at<float>(i, j));
    }

    printf("\n");
  }
}

void dumpDecomposition(const HomographyDecomposition& d)
{
  printf("d = %f, score = %d\n", d.dp, d.score);
  dumpFltMat("R", d.R);
  dumpFltMat("T", d.T);
}

void dumpDecompositions(const vector<HomographyDecomposition>& decompositions)
{
  for(size_t i = 0; i < decompositions.size(); i++)
  {
    printf("Decomposition %d:\n", (int)i);
    dumpDecomposition(decompositions[i]);
  }
}

void keyPoints2Point2f(const vector<KeyPoint>& src, vector<Point2f>& dst)
{
  dst.resize(src.size());
  for(size_t i = 0; i < src.size(); i++)
  {
    dst[i] = src[i].pt;
  }
}

void matchesFromIndices(const vector<KeyPoint>& src1, const vector<KeyPoint>& src2, const vector<cv::DMatch>& indices,
                        vector<Point2f>& dst1, vector<Point2f>& dst2)
{
  dst1.clear();
  dst2.clear();
  for(size_t i = 0; i < indices.size(); i++)
  {
    dst1.push_back(src1[indices[i].queryIdx].pt);
    dst2.push_back(src2[indices[i].trainIdx].pt);
  }
}

void matchesFromIndices(const vector<Point2f>& set1, const vector<Point2f>& set2, const vector<int>& indices,
                        vector<Point2f>& points1, vector<Point2f>& points2)
{
  points1 = set1;
  vectorSubset(set2, indices, points2);
}

void matchesFromIndices(const vector<KeyPoint>& _set1, const vector<KeyPoint>& _set2, const vector<int>& indices,
                        vector<Point2f>& points1, vector<Point2f>& points2)
{
  assert(_set1.size() == indices.size());

  vector<Point2f> set1, set2;
  keyPoints2Point2f(_set1, set1);
  keyPoints2Point2f(_set2, set2);

  matchesFromIndices(set1, set2, indices, points1, points2);
}

// Computes homography from its decomposition. Can be used for testing
// decomposition integrity
Mat homographyFromDecomposition(const HomographyDecomposition& decomposition)
{
  Mat Rd = decomposition.R*decomposition.dp + decomposition.T*decomposition.Np.t();
  return Rd;
}

//! Calculates 8 possible decompositions of a homography
//! @param H Homography matrix
//! @param hd Output decompositions
bool homographyDecompose(const Mat& intrinsics, const Mat& _H, vector<HomographyDecomposition>& hd)
{
  hd.clear();

  // compensate for intrinsics
  Mat H = intrinsics.inv()*_H*intrinsics;

  cv::SVD svd(H);
  double d1 = fabs(svd.w.at<float>(0, 0));
  double d2 = fabs(svd.w.at<float>(1, 0));
  double d3 = fabs(svd.w.at<float>(2, 0));

  Mat u, v;
  svd.u.convertTo(u, CV_32F);

  Mat(svd.vt.t()).convertTo(v, CV_32F);

  double s = determinant(svd.u)*determinant(svd.vt.t());

  double dPrime_PM = d2;

  int nCase;
  if(d1 != d2 && d2 != d3)
    nCase = 1;
  else if( d1 == d2 && d2 == d3)
    nCase = 3;
  else
    nCase = 2;

  if(nCase != 1)
  {
    std::cout << "  Homographyinit: This motion case is not implemented or is degenerate. Try again. " << std::endl;
    return false;
  }

  double x1_PM;
  double x2;
  double x3_PM;

  // All below deals with the case = 1 case.
  // Case 1 implies (d1 != d3)
  { // Eq. 12
    x1_PM = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
    x2    = 0;
    x3_PM = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
  };

  double e1[4] = {1.0,-1.0,1.0,-1.0};
  double e3[4] = {1.0, 1.0, -1.0,-1.0};

  Mat Np(3, 1, CV_32F);
  HomographyDecomposition decomposition;

  // Case 1, d' > 0:
  decomposition.dp = s * dPrime_PM;
  for(int signs=0; signs<4; signs++)
  {
    // Eq 13
    decomposition.Rp = Mat::eye(3, 3, CV_32F);
    double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
    double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
    decomposition.Rp.at<float>(0, 0) = dCosTheta;
    decomposition.Rp.at<float>(0, 2) = -dSinTheta;
    decomposition.Rp.at<float>(2, 0) = dSinTheta;
    decomposition.Rp.at<float>(2, 2) = dCosTheta;

    // Eq 14
    decomposition.Tp.at<float>(0, 0) = (d1 - d3) * x1_PM * e1[signs];
    decomposition.Tp.at<float>(1, 0) = 0.0;
    decomposition.Tp.at<float>(2, 0) = (d1 - d3) * -x3_PM * e3[signs];

    Np.at<float>(0, 0) = x1_PM * e1[signs];
    Np.at<float>(1, 0) = x2;
    Np.at<float>(2, 0) = x3_PM * e3[signs];

    decomposition.R = s * u * decomposition.Rp * v.t();
    decomposition.T = u * decomposition.Tp;
    decomposition.Np = v * Np;

#if defined(DEBUG_CONSOLE)
    printf("Dumping decomposition %d\n", (int)hd.size());
    dumpDecomposition(decomposition);
    Mat L = decomposition.Rp*dPrime_PM + decomposition.Tp*Np.t();
    dumpFltMat("L", L);
#endif //DEBUG_CONSOLE

    hd.push_back(decomposition);
  }

  // Case 1, d' < 0:
  decomposition.dp = s * -dPrime_PM;
  for(int signs=0; signs<4; signs++)
  {
    // Eq 15
    decomposition.Rp = -Mat(Mat::eye(3, 3, CV_32F));
    double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
    double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
    decomposition.Rp.at<float>(0, 0) = dCosPhi;
    decomposition.Rp.at<float>(0, 2) = dSinPhi;
    decomposition.Rp.at<float>(2, 0) = dSinPhi;
    decomposition.Rp.at<float>(2, 2) = -dCosPhi;

    // Eq 16
    decomposition.Tp.at<float>(0, 0) = (d1 + d3) * x1_PM * e1[signs];
    decomposition.Tp.at<float>(1, 0) = 0.0;
    decomposition.Tp.at<float>(2, 0) = (d1 + d3) * x3_PM * e3[signs];

    Np.at<float>(0, 0) = x1_PM * e1[signs];
    Np.at<float>(1, 0) = x2;
    Np.at<float>(2, 0) = x3_PM * e3[signs];

    decomposition.R = s * u * decomposition.Rp * v.t();
    decomposition.T = u * decomposition.Tp;
    decomposition.Np = v * Np;

#if defined(DEBUG_CONSOLE)
    printf("Dumping decomposition %d\n", (int)hd.size());
    dumpDecomposition(decomposition);
    Mat L = decomposition.Rp*dPrime_PM + decomposition.Tp*Np.t();
    dumpFltMat("L", L);
#endif //DEBUG_CONSOLE

    hd.push_back(decomposition);
  }

  return true;
}

//! Computes homography inliers
//! @param src1 First set of points
//! @param src2 Second set of points
//! @param H 3x3 homography matrix
//! @param inliers1 First set of inliers
//! @param inliers2 Second set of inliers
//! @param maxProjError Maximum reprojection error of an inlier
void computeHomographyInliers(const vector<Point2f>& src1, const vector<Point2f>& src2, const Mat& H,
                              vector<Point2f>& inliers1, vector<Point2f>& inliers2, float maxProjError = 2.0f)
{
  vector<Point2f> src1_mapped;
  src1_mapped.resize(src1.size());
  Mat _src1_mapped(src1_mapped);
  perspectiveTransform(Mat(src1), _src1_mapped, H);
  for(size_t i = 0; i < src1.size(); i++)
  {
    float dist = norm(src2[i] - src1_mapped[i]);
    if(dist < maxProjError)
    {
      inliers1.push_back(src1[i]);
      inliers2.push_back(src2[i]);
    }
  }
}

//! Calculates the epipolar reprojection error pl*E*pr
//! @param essential Essential matrix
//! @param pl A point from the first image
//! @param pr A point from the second image
static double SampsonusError(const Mat& essential, Point2f pl, Point2f pr)
{
  Point3f Pl(pl.x, pl.y, 1.0f);
  Point3f Pr(pr.x, pr.y, 1.0f);
  Mat _Pl(1, 3, CV_32F, &Pl);
  Mat _Pr(3, 1, CV_32F, &Pr);

  //    float error = Mat(_Pl * essential * _Pr).at<float>(0, 0);

  Point3f fPr = *(Point3f*)Mat(essential * _Pr).ptr(0);
  Point3f fPl = *(Point3f*)Mat(essential.t() * _Pl.t()).ptr(0);
  float error = Pl.dot(fPr);

  Point2f fPrPlane(fPr.x, fPr.y);
  Point2f fPlPlane(fPl.x, fPl.y);

  return error*error/(fPrPlane.dot(fPrPlane) + fPlPlane.dot(fPlPlane));
}

//! Computes epipolar inliers
void computeEpipolarInliers(const Mat& essential, const vector<Point2f>& points1, const vector<Point2f>& points2,
                            vector<bool>& inliers, double maxError)
{
  assert(points1.size() == points2.size());

  inliers.resize(points1.size());
  const double maxErrorSquared = maxError*maxError;
  for(size_t i = 0; i < points1.size(); i++)
  {
    double error = SampsonusError(essential, points2[i], points1[i]);
    //        printf("i = %d, p1 = %f %f, p2 = %f %f, Epipolar error = %f\n", (int)i, points1[i].x, points1[i].y, points2[i].x, points2[i].y, error);
    if(error < maxErrorSquared)
    {
      inliers[i] = true;
    }
    else
    {
      inliers[i] = false;
    }
  }
}


void filterDecompositionsVisibility(vector<HomographyDecomposition>& decompositions, const Mat& H, const vector<Point2f>& inliers1, const vector<Point2f>& inliers2)
{
  assert(decompositions.size() == 8);

  // First, filter out 4 solutions using a visibility constraint (First Eq from Prop.4)
  for(unsigned int i = 0; i < decompositions.size(); i++)
  {
    HomographyDecomposition &decomposition = decompositions[i];
    int nPositive = 0;
    for(unsigned int m=0; m<inliers1.size(); m++)
    {
      Point3f inlier(inliers1[m].x, inliers1[m].y, 1.0f);
      Mat _inlier(3, 1, CV_32F, &inlier);
      double dVisibilityTest = Mat(H*_inlier).at<float>(2, 0)/decomposition.dp;

      //            printf("inlier1: %f %f, inlier2: %f %f\n", inliers1[m].x, inliers1[m].y, inliers2[m].x, inliers2[m].y);
      //            printf("H*inlier1.z = %f\n", Mat(H*_inlier).at<float>(2, 0));
      if(dVisibilityTest > 0.0)
        nPositive++;
    };
    decomposition.score = -nPositive;

    //        printf("Decomposition %d: score %d\n", i, decomposition.score);
  }

  sort(decompositions.begin(), decompositions.end());

  // include at least 4 decompositions and filter out the rest with higher score
  size_t k;
  for(k = 3; k < decompositions.size(); k++)
  {
    if(decompositions[k].score > decompositions[3].score)
      break;
  }
  decompositions.resize(k);

#if defined(DEBUG_CONSOLE)
  printf("\n\nDumping decompositions after step 1:\n");
  dumpDecompositions(decompositions);
#endif //DEBUG_CONSOLE

  // Now filter out two more solutions using visibility constraint (second Eq from Prop 4)
  for(unsigned int i=0; i<decompositions.size(); i++)
  {
    HomographyDecomposition &decomposition = decompositions[i];
    int nPositive = 0;
    for(unsigned int m=0; m<inliers1.size(); m++)
    {
      Point3f inlier(inliers1[m].x, inliers1[m].y, 1);
      double dVisibilityTest = inlier.dot(*(Point3f*)decomposition.Np.ptr(0))/decomposition.dp;
      if(dVisibilityTest > 0.0)
        nPositive++;
    };
    decomposition.score = -nPositive;
  }

  sort(decompositions.begin(), decompositions.end());

#if defined(DEBUG_CONSOLE)
  printf("\n\nDumping decompositions after step 2 before filtering:\n");
  dumpDecompositions(decompositions);
#endif //DEBUG_CONSOLE

  // include at least 2 decompositions and filter out the rest with higher score
  for(k = 1; k < decompositions.size(); k++)
  {
    if(decompositions[k].score > decompositions[1].score)
      break;
  }
  decompositions.resize(k);

#if defined(DEBUG_CONSOLE)
  printf("\n\nDumping decompositions after step 2 filtering:\n");
  dumpDecompositions(decompositions);
#endif //DEBUG_CONSOLE

}

Mat calcEssentialMatrix(const Mat& intrinsics_inv, const Mat& R, const Mat& T)
{
  Mat essential(3, 3, CV_32F);
#if 0 // possible bug
  for(int j = 0; j < 3; j++)
  {
    Point3f R_row = *(Point3f*)R.ptr(j);
    Point3f t = *(Point3f*)T.ptr(0);
    *(Point3f*)essential.ptr(j) = crossProduct(R_row, t);
  }
#else
#if 0 // also not correct
  for(int j = 0; j < 3; j++)
  {
    Mat Rt = R.t();
    Point3f R_col = *(Point3f*)Rt.ptr(j);
    Point3f t = *(Point3f*)T.ptr(0);
    *(Point3f*)essential.ptr(j) = crossProduct(t, R_col);
  }

  essential = essential.t();
#else
  Mat Rt = R.t();
  Mat Tx = Rt*T;
  for(int j = 0; j < 3; j++)
  {
    Point3f R_row = *(Point3f*)R.ptr(j);
    Point3f tx = *(Point3f*)Tx.ptr(0);
    *(Point3f*)essential.ptr(j) = crossProduct(R_row, tx);
  }
#endif
#endif

  essential = intrinsics_inv.t()*essential*intrinsics_inv;
  //    dumpFltMat("intrinsics @ essential matrix", intrinsics_inv);
  return essential;
}

double avgSampsonusError(const Mat& essential, const vector<Point2f>& points1, const vector<Point2f>& points2, double max_error, bool verbose)
{
  assert(points1.size() == points2.size());

  double error_limit  = max_error*max_error*4;
  double sum_error = 0;
  for(unsigned int m = 0; m < points1.size(); m++ )
  {
    double d = SampsonusError(essential, points2[m], points1[m]);
    if(verbose)
    {
      printf("%d %f\n", m, d);
    }
    if(d > error_limit)
      d = error_limit;
    sum_error += d;
  }

  return sum_error/points1.size();
}

double filterDecompositionsZ(vector<HomographyDecomposition>& decompositions, const vector<Point2f>& points1, const vector<Point2f>& points2,
                             const Mat& intrinsics, const Mat& intrinsics_inv)
{
  printf("Called filterDecompositionsZ\n");
  int max_idx = -1;
  int max_score = -1;
  for(size_t i = 0; i < decompositions.size(); i++)
  {
    vector<Point3f> cloud;
    vector<bool> valid;
    reprojectPoints(intrinsics, decompositions[i].R, decompositions[i].T, points1, points2, cloud, valid);

    const double reprojectionError = 1.0f;
    filterInliers(cloud, points1, points2, decompositions[i].R, decompositions[i].T, intrinsics, reprojectionError, valid);

    int score = 0;
    for(size_t j = 0; j < valid.size(); j++) score += int(valid[j]);
    printf("  decomposition %d: score = %d\n", (int)i, score);
    if(score > max_score)
    {
      max_score = score;
      max_idx = i;
    }
  }

  //  printf("decompositions size: %d, chosen %d with score %d\n", (int)decompositions.size(), max_idx, max_score);
  vector<HomographyDecomposition> filtered;
  filtered.push_back(decompositions[max_idx]);
  decompositions = filtered;

  Mat essential = calcEssentialMatrix(intrinsics_inv, decompositions[0].R, decompositions[0].T);
  double epipolar_error = avgSampsonusError(essential, points1, points2);
  return epipolar_error;
}

double filterDecompositionsEpipolar(const Mat& intrinsics, const Mat& intrinsics_inv, vector<HomographyDecomposition>& decompositions,
                                    const vector<Point2f>& points1, const vector<Point2f>& points2)
{
  // According to Faugeras and Lustman, ambiguity exists if the two scores are equal
  // but in practive, better to look at the ratio!
  //  printf("\nEpipolar decompositions:\n");
  //  dumpDecompositions(decompositions);
  double ratio = (double) decompositions[1].score / (double) decompositions[0].score;

  if(ratio < 0.9) // no ambiguity!
  {
//    printf("Returning with ratio = %f\n", ratio);
    decompositions.erase(decompositions.begin() + 1);
  }

  vector<double> epipolar_error;
  epipolar_error.resize(decompositions.size());
  for(size_t i = 0; i < decompositions.size(); i++)
  {
    Mat essential = calcEssentialMatrix(intrinsics_inv, decompositions[i].R, decompositions[i].T);

    epipolar_error[i] = avgSampsonusError(essential, points1, points2);
    //        printf("%f ", epipolar_error[i]);
  }
  //    printf("\n");

  // filter out higher epipolar

  if(decompositions.size() == 1)
  {
    return epipolar_error[0];
  }

//  printf("Decomposition epipolar errors: %f %f\n", epipolar_error[0], epipolar_error[1]);
#if 0
  const double minAvgEpipolarError = 1.0;
  if( epipolar_error[0] < minAvgEpipolarError &&
      epipolar_error[1] < minAvgEpipolarError) //at least two small epipolar errors
  {
    // filter decompositions based on epipolar error
    vector<HomographyDecomposition> _decompositions;
    for(size_t i = 0; i < epipolar_error.size(); i++)
    {
      if(epipolar_error[i] < minAvgEpipolarError)
      {
        _decompositions.push_back(decompositions[i]);
      }
    }
    decompositions = _decompositions;
    double error = filterDecompositionsZ(decompositions, points1, points2, intrinsics, intrinsics_inv);
    return error;
  }
#endif

  // choose the smallest epipolar error
  vector<double>::iterator maxIt = std::max_element(epipolar_error.begin(), epipolar_error.end());
  double error = *maxIt;
  vector<HomographyDecomposition> _decompositions;
  _decompositions.push_back(decompositions[maxIt - epipolar_error.begin()]);
  decompositions = _decompositions;

  return error;
}

//! Filters out decompositions
//! @param decompositions A set of decompositions of a homography matrix
//! @param H Homography patrix
//! @param points1 First set of points
//! @param points2 Second set of points
//! @param inliers1 First set of inliers
//! @param inliers2 Second set of inliers
void selectDecomposition(vector<HomographyDecomposition>& decompositions, const Mat& H, const vector<Point2f>& points1, const vector<Point2f>& points2,
                         const vector<Point2f>& inliers1, const vector<Point2f>& inliers2, const Mat& intrinsics, const Mat& intrinsics_inv)
{
  assert(decompositions.size() == 8);
  filterDecompositionsVisibility(decompositions, H, inliers1, inliers2);

  assert(decompositions.size() == 2);
  filterDecompositionsEpipolar(intrinsics, intrinsics_inv, decompositions, points1, points2);
}

void logDecompositions(std::string filename, const vector<HomographyDecomposition>& decompositions)
{
  FileStorage fs(filename, FileStorage::WRITE);
  for(size_t i = 0; i < decompositions.size(); i++)
  {
    fs << std::string("R") << decompositions[i].R;
    fs << std::string("T") << decompositions[i].T;
  }
}

bool operator<(const HomographyDecomposition lhs, const HomographyDecomposition rhs)
{
  return lhs.score < rhs.score;
}

//selects a subset of indices without replacement in the region [0, max_index-1].
void sample(int max_index, int count, vector<int>& sample_indices)
{
  sample_indices.clear();
  for(int i = 0; i < count; i++)
  {
    int index;
    while(1)
    {
      index = rand()%max_index;
      if(std::find(sample_indices.begin(), sample_indices.end(), index) == sample_indices.end())
      {
        break;
      }
    }

    sample_indices.push_back(index);
    if((int)sample_indices.size() == max_index)
    {
      break;
    }
  }
}

// selects four random pair of points and runs homography calculation on them
Mat randomHomography(const vector<Point2f>& points1, const vector<Point2f>& points2,
                     vector<Point2f>& sample1, vector<Point2f>& sample2)
{
  vector<int> indices;

  sample(points1.size(), 4, indices);
  vectorSubset(points1, indices, sample1);
  vectorSubset(points2, indices, sample2);

  Mat _H = getPerspectiveTransform(&sample1.front(), &sample2.front());
  Mat H;
  _H.convertTo(H, CV_32F);
  return H;
}

double SFMwithSBA(const Mat& intrinsics, const vector<KeyPoint>& set1, const vector<KeyPoint>& set2,
                  const vector<int>& indices, Mat& rvec, Mat& T, double reprojectionError)
{
  vector<Point2f> points1, points2;
  matchesFromIndices(set1, set2, indices, points1, points2);

  return SFMwithSBA(intrinsics, points1, points2, rvec, T, reprojectionError);
}

void _filterInliers(const vector<Point3f>& obj_pts, const vector<Point2f>& img_pts,
                    const Mat& R, const Mat& T, const Mat& intrinsics, const Mat& distortion,
                    double reprojectionError, vector<bool>& valid)
{
  assert(valid.size() == img_pts.size());
  vector<Point2f> img_pts_proj;
  img_pts_proj.resize(img_pts.size());
  Mat _R, _T;
  R.convertTo(_R, CV_64F);
  T.convertTo(_T, CV_64F);
  projectPoints(Mat(obj_pts), _R, _T, intrinsics, distortion, img_pts_proj);

  int count = 0;
  for(size_t i = 0; i < img_pts.size(); i++)
  {
    if(!valid[i]) continue;

    float error = norm(img_pts[i] - img_pts_proj[i]);
    if(error > reprojectionError)
    {
      valid[i] = false;
    }
    else
    {
      count++;
    }
  }
  //    std::cout << "Total number of valid points: " << count << std::endl;
}

void filterInliers(const vector<Point3f>& obj_pts, const vector<Point2f>& img1_pts, const vector<Point2f>& img2_pts,
                   const Mat& R, const Mat& T, const Mat& intrinsics, double reprojectionError, vector<bool>& valid)
{
  Mat distortion = Mat::zeros(5, 1, CV_32F);
  Mat R0 = Mat::eye(3, 3, CV_32F);
  Mat T0 = Mat::zeros(3, 1, CV_32F);

  _filterInliers(obj_pts, img1_pts, R0, T0, intrinsics, distortion, reprojectionError, valid);
  _filterInliers(obj_pts, img2_pts, R, T, intrinsics, distortion, reprojectionError, valid);
}

double SFMwithSBA(const Mat& intrinsics, vector<Point2f>& points1, vector<Point2f>& points2,
                  Mat& rvec, Mat& T, double reprojectionError)
{
  if(points1.size() < 4 || points2.size() < 4)
  {
    printf("SFMwithSBA called with %d points, exiting...\n", (int)points1.size());
    return -1.0;
  }

  Mat R, H;
  double error = SFM(intrinsics, points1, points2, R, T);
  //    printf("SFM completed with reprojection error %f\n", error);
  T = T*10.0;
  Mat _r(3, 1, CV_32F);
  Rodrigues(R, _r);
  dumpFltMat("SFM returned r", _r);
  dumpFltMat("SFM return T", T);

  Mat essential = calcEssentialMatrix(intrinsics.inv(), R, T);
  vector<bool> inliers;
  computeEpipolarInliers(essential, points1, points2, inliers, 10.0);
  filterVector(points1, inliers);
  filterVector(points2, inliers);
  //    filterVector(points, inliers);
  int inliersSum = 0;
  for(size_t i = 0; i < inliers.size(); i++) {inliersSum += inliers[i] ? 1 : 0;};
  cout << "The number of epipolar inliers: " << inliersSum << endl;

  vector<Point3f> cloud;
  vector<bool> valid;
  reprojectPoints(intrinsics, R, T, points1, points2, cloud, valid);

  printf("%d points before filtering\n", (int)cloud.size());
  filterVector(points1, valid);
  filterVector(points2, valid);
  //    filterVector(points, valid);
  filterVector(cloud, valid);

  printf("%d points left after filtering\n", (int)cloud.size());
  //    float error0 = calcScaledPointCloudDistance(cloud, points);

  //    Mat rvec;
  Rodrigues(R, rvec);

  sba(intrinsics, rvec, T, cloud, points1, points2);
  findNaNPoints(cloud, valid);
  filterVector(points1, valid);
  filterVector(points2, valid);
  filterVector(cloud, valid);
  //    filterVector(points, valid);

  //    float error1 = calcScaledPointCloudDistance(cloud, points);

  //    dumpFltMat("rvec", rvec);
  //    dumpFltMat("tvec", T);

  //    printf("%d points left after sba\n3D error after SFM: %f\n 3D error after sba: %f\n",
  //        cloud.size(), error0, error1);
  printf("%d points left after sba\n", (int)cloud.size());

  return error;
}

double SFM(const Mat& intrinsics, const vector<KeyPoint>& set1, const vector<KeyPoint>& set2,
           const vector<int>& indices, Mat& R, Mat& T, double reprojectionError)
{
  vector<Point2f> points1, points2;
  matchesFromIndices(set1, set2, indices, points1, points2);

  return SFM(intrinsics, points1, points2, R, T, reprojectionError);
}

double SFM(const Mat& intrinsics, const vector<Point2f>& points1, const vector<Point2f>& points2,
           Mat& R, Mat& T, double reprojectionError)
{
  vector<Point2f> full1 = points1, full2 = points2;
  //    points1.resize(100);
  //    points2.resize(100);

  Mat intrinsics_inv = intrinsics.inv();

  // ransac iteration
  const int ransac_count = 10000;
  double min_error = 1e10;

  const int min_acceptable_inlier_count = points1.size()*0.8;
  const double min_acceptable_error = 1.0;

  HomographyDecomposition best_decomposition;
  vector<HomographyDecomposition> best_decompositions;
  int maxInlierCount = 0;
  for(int i = 0; i < ransac_count; i++)
  {
//    int64 _t1 = cvGetTickCount();
    vector<Point2f> sample1, sample2;
    Mat H = randomHomography(points1, points2, sample1, sample2);

    //        dumpFltMat("H", H);

    vector<HomographyDecomposition> decompositions, _decompositions;
    bool ret = homographyDecompose(intrinsics, H, decompositions);
    _decompositions = decompositions;
    if(!ret) continue;

    //        printf("\nDecompositions:\n");
    //        dumpDecompositions(decompositions);
    //        printf("\n\n");

//    int64 _t2 = cvGetTickCount();

    // compute planar inliers
    vector<Point2f> inliers1;// = sample1;
    vector<Point2f> inliers2;// = sample2;
    computeHomographyInliers(points1, points2, H, inliers1, inliers2, reprojectionError);
    maxInlierCount = max(maxInlierCount, (int)inliers1.size());

    // filter out 6 decompositions using visibility constraint
    filterDecompositionsVisibility(decompositions, H, inliers1, inliers2);

//    int64 _t3 = cvGetTickCount();

    //        printf("\nEpipolar decompositions:\n");
    //        dumpDecompositions(decompositions);
    //        printf("\n\n");

    // filter out the rest two decompositions using epipolar reprojection error
    double error = filterDecompositionsEpipolar(intrinsics, intrinsics_inv, decompositions, points1, points2);

    //        dumpDecompositions(decompositions);
    //        printf("Error = %f\n", error);

//    int64 _t4 = cvGetTickCount();

    /*        printf("Time elapsed in ms:\n homography %f\n inliers/visiblility %f\n epipolar error %f\n",
                1e-3*(_t2 - _t1)/cvGetTickFrequency(), 1e-3*(_t3 - _t2)/cvGetTickFrequency(),
                1e-3*(_t4 - _t3)/cvGetTickFrequency());*/

    if(i % 1000 == 0)
    {
      //            printf("Iteration %d: minimum error %f\n", i, min_error);
    }
    if(error < min_error)
    {
      min_error = error;
      best_decomposition = decompositions[0];
      best_decompositions = _decompositions;

      if(error < min_acceptable_error && (int)inliers1.size() > min_acceptable_inlier_count)
      {
        printf("Finishing after iteration %d, inlier count %d\n", i, (int)inliers1.size());
        //                dumpFltMat("H", H);
        break;
      }
    }
  }

#if defined(DEBUG_CONSOLE)
  cout << "Max inlier count " << maxInlierCount << endl;

  dumpDecompositions(best_decompositions);
#endif //DEBUG_CONSOLE

  R = best_decomposition.R;
  T = best_decomposition.T;

#if defined(DEBUG_CONSOLE)
  dumpFltMat("best R", R);
  dumpFltMat("best T", T);
#endif //DEBUG_CONSOLE

  Mat E = calcEssentialMatrix(intrinsics_inv, R, T);
  double final_error = avgSampsonusError(E, full1, full2, std::numeric_limits<double>::max());
  printf("Final error: %f\n", final_error);

  return min_error;
}

void planarSFM(Mat& intrinsics, const vector<KeyPoint>& set1, const vector<KeyPoint>& set2, const vector<int>& indices,
               Mat& H, Mat& R, Mat& T, double reprojectionError)
{
  Mat intrinsics_inv = intrinsics.inv();

  // Extract correspondences
  vector<Point2f> points1, points2;
  matchesFromIndices(set1, set2, indices, points1, points2);

  // find homography
  vector<Point2f> points1s, points2s;
  points1s = points1;
  points2s = points2;
  points1s.resize(4);
  points2s.resize(4);
  Mat _H = findHomography(Mat(points1s), Mat(points2s), CV_RANSAC, reprojectionError);
  _H.convertTo(H, CV_32F);

  // Compute inliers
  vector<Point2f> inliers1, inliers2;
  computeHomographyInliers(points1, points2, H, inliers1, inliers2, reprojectionError);

  // decompose homography to obtain 8 possible solutions
  vector<HomographyDecomposition> decompositions;
  homographyDecompose(intrinsics, H, decompositions);

  // select one solution from 8
  selectDecomposition(decompositions, H, points1, points2, inliers1, inliers2, intrinsics, intrinsics_inv);

  assert(decompositions.size() == 1);
  R = decompositions[0].R;
  T = decompositions[0].T;
}

void initNode(const Mat& intrinsics, const Mat& rvec, const Mat& tvec, sba::Node& node)
{
  // rvec to quaternion
  double alpha = norm(rvec);
  double s;
  if(fabs(alpha) < std::numeric_limits<double>::epsilon())
  {
    s = 0.5;
  }
  else
  {
    s = sin(alpha*0.5)/alpha;
  }

  node.qrot = Vector4d(rvec.at<float>(0, 0)*s, rvec.at<float>(0, 1)*s, rvec.at<float>(0, 2)*s, cos(alpha*0.5));
  node.trans = Vector4d(tvec.at<float>(0, 0), tvec.at<float>(0, 1), tvec.at<float>(0, 2), 1);
  node.setTransform();

  fc::CamParams cp;
  cp.fx = intrinsics.at<float>(0, 0);
  cp.fy = intrinsics.at<float>(1, 1);
  cp.cx = intrinsics.at<float>(0, 2);
  cp.cy = intrinsics.at<float>(1, 2);
  // TBD should I set tx here?
      node.setKcam(cp);

      cout << "Node projection matrix" << endl << node.w2i << endl;

      node.setDri();

}

Point3f mult(const Mat& M, const Point3f& p)
{
  Mat r = M*Mat(3, 1, CV_32F, const_cast<Point3f*>(&p));
  return *r.ptr<Point3f>(0);
}

Point3f multh(const Mat& M, const Point2f& p)
{
  Point3f ph(p.x, p.y, 1.0f);
  return mult(M, ph);
}

void findRayIntersection(Point3f k1, Point3f b1, Point3f k2, Point3f b2, Point3f& p)
{
  Point3f n = crossProduct(k1, k2);

  // find the closest point in 3D between two rays
  Mat A(3, 3, CV_32F), B(3, 1, CV_32F);
  *A.ptr<Point3f>(0) = -k1;
  *A.ptr<Point3f>(1) = k2;
  *A.ptr<Point3f>(2) = -n;
  A = A.t();
  *B.ptr<Point3f>(0) = b1 - b2;

  Mat X;
  solve(A, B, X);

  float s1 = X.at<float>(0, 0);
  float s2 = X.at<float>(1, 0);
  /*
    Point3f cross1 = k1*s1 + b1;
    Point3f cross2 = k2*s2 + b2;

    printf("k1: %f %f %f\n", k1.x, k1.y, k1.z);
    printf("k2: %f %f %f\n", k2.x, k2.y, k2.z);
    printf("b1: %f %f %f\n", b1.x, b1.y, b1.z);
    printf("b2: %f %f %f\n", b2.x, b2.y, b2.z);
    printf("s1: %f, s2: %f\n", s1, s2);
    printf("cross1: %f %f %f\n", cross1.x, cross1.y, cross1.z);
    printf("cross2: %f %f %f\n", cross2.x, cross2.y, cross2.z);
   */
  p = (k1*s1 + b1 + k2*s2 + b2)*0.5f;
}

//! reprojects points to 3d
//! p1 and p2 are uniform coordinates in 2d, identity intrinsics matrix assumed
void reprojectPoint(const Mat& R, const Mat& T, Point3f p1, Point3f p2, Point3f& p)
{
  // find two rays in the form p(s) = k*s + b
  Point3f k1 = p1;
  Point3f b1(0.0f, 0.0f, 0.0f);
  Point3f k2 = mult(R.t(), p2);
  Mat tnew = -Mat(R.t()*T);
  Point3f b2 = *tnew.ptr<Point3f>(0);

  //    printf("k1 = %f %f %f, b1 = %f %f %f, k2 = %f %f %f, b2 = %f %f %f\n", k1.x, k1.y, k1.z,
  //           b1.x, b1.y, b1.z, k2.x, k2.y, k2.z, b2.x, b2.y, b2.z);

  findRayIntersection(k1, b1, k2, b2, p);
}

void reprojectPoints(const Mat& intrinsics, const Mat& R, const Mat& T, const vector<Point2f>& p1, const vector<Point2f>& p2,
                     vector<Point3f>& p, vector<bool>& valid)
{
  Mat intrinsics_inv = intrinsics.inv();

  //    dumpFltMat("Reprojecting with R", R);
  //    dumpFltMat("Reprojecting with T", T);

  p.resize(p1.size());
  valid.resize(p1.size());
  assert(p1.size() == p2.size());
  for(size_t i = 0; i < p1.size(); i++)
  {
    Point3f rp1 = multh(intrinsics_inv, p1[i]);
    Point3f rp2 = multh(intrinsics_inv, p2[i]);

    reprojectPoint(R, T, rp1, rp2, p[i]);
    if(p[i].z < 0)
    {
      // filter this point out
      valid[i] = false;
//      printf("Filtering point (%f %f), (%f %f), \n(%f %f), (%f %f), (%f %f %f) out\n",
//             p1[i].x, p1[i].y, p2[i].x, p2[i].y, rp1.x, rp1.y, rp2.x, rp2.y, p[i].x, p[i].y, p[i].z);
    }
    else
    {
      valid[i] = true;
    }
  }
}

inline double calcNodeErr(sba::Proj& prj, const sba::Node &nd, const sba::Point &pt)
{
  Eigen::Vector3d p1 = nd.w2i * pt;
  prj.err = p1.head(2)/p1(2);
  if (p1(2) <= 0.0)
  {
    prj.err = Vector3d(0.0,0.0,0.0);
    return 0.0;
  }
  //    printf("Projection: %f %f, gt: %f %f, ", prj.err(0), prj.err(1), prj.kp(0), prj.kp(1));
  prj.err -= prj.kp;
  //  printf("dist: %f\n", prj.err.squaredNorm());
  return prj.err.squaredNorm();
}

double calcCamProjCost(sba::SysSBA& sba, int cam)
{
  double cost = 0.0;
  printf("sba.tracks.size = %d\n", (int)sba.tracks.size());
  for(size_t i=0; i<sba.tracks.size(); i++)
  {
    sba::ProjMap &prjs = sba.tracks[i].projections;
    if (prjs.size() == 0) continue;
    for(sba::ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
    {
      sba::Proj &prj = itr->second;
      if (!prj.isValid) continue;
      //            printf("ndi = %d\n", prj.ndi);
      if(prj.ndi != cam) continue;
      double err = calcNodeErr(prj, sba.nodes[prj.ndi], sba.tracks[i].point);
      //          if (err < 0.0)
      //            prj.isValid = false;
      //          else
      cost += err;
      //            printf("prj.pti = %d, err = %f, cost = %f\n", prj.pti, err, cost);
    }
  }

  printf("costProj done \n");
  return cost/sba.tracks.size();
}

void sba(const Mat& intrinsics, Mat& rvec, Mat& tvec, vector<Point3f>& points, const vector<Point2f>& points1, const vector<Point2f>& points2)
{
  printf("sba got %d points\n", (int)points.size());
  // system
  sba::SysSBA sba0;
  sba0.verbose = 0;

  // set up nodes
  sba::Node nd0,nd1;
  Mat rvec0 = Mat::zeros(3, 1, CV_32F);
  Mat tvec0 = Mat::zeros(3, 1, CV_32F);
  initNode(intrinsics, rvec0, tvec0, nd0);
  nd0.isFixed = true;

  Mat sbaRvec = rvec;
  Mat sbaTvec = tvec;
  Mat sbaR;
  Rodrigues(sbaRvec, sbaR);
  sbaR = sbaR.t();
  sbaTvec = -sbaR*tvec;
  sbaRvec = -sbaRvec;

  initNode(intrinsics, sbaRvec, sbaTvec, nd1);
  nd1.isFixed = false;

  //    dumpFltMat("Initialized with rvec", sbaRvec);
  //    dumpFltMat("Initialized with tvec", sbaTvec);

  sba0.nodes.push_back(nd0);
  sba0.nodes.push_back(nd1);

  // set up projections
  for (size_t i=0; i<points.size(); i++)
  {
    // add point
    sba0.addPoint(Vector4d(points[i].x, points[i].y, points[i].z, 1.0));
    //        printf("%f %f %f\n", points[i].x, points[i].y, points[i].z);

    Vector2d p1(points1[i].x, points1[i].y);
    Vector2d p2(points2[i].x, points2[i].y);

    sba0.addMonoProj(0, (int)i, p1);
    sba0.addMonoProj(1, (int)i, p2);

  }

  printf("Added %d points, %d tracks\n", (int)sba0.tracks.size(), (int)sba0.tracks.size());

  double error1 = calcCamProjCost(sba0, 0);
  double error2 = calcCamProjCost(sba0, 1);
  printf("Errors after sba initialization: %f %f\n", error1, error2);

  //    sba.A.setZero(6, 6);
  sba0.nFixed = 1;
  sba0.printStats();
  // cout << "oldpoints.size() = " << sba0.oldpoints.size() << endl;
  printf("sba pointer: %p\n", &sba0);
  sba0.doSBA(5,10e-5,false);
  int nbad = sba0.removeBad(2.0);
  cout << endl << "Removed " << nbad << " projections > 2 pixels error" << endl;
  sba0.doSBA(5,10e-5,false);

  //        cout << endl << sba.nodes[1].trans.transpose().start(3) << endl;
  Eigen::Vector3d trans = sba0.nodes[1].trans.head(3);
  printf("trans = %f %f %f\n", trans(0), trans(1), trans(2));
  // do the convertion manually as there are
  *sbaTvec.ptr<Point3f>(0) = Point3f(trans(0), trans(1), trans(2));

  Quaterniond q1;
  q1 = sba0.nodes[1].qrot;
  Matrix3d rot = q1.toRotationMatrix();
  for(int i = 0; i < 3; i++)
  {
    *sbaR.ptr<Point3f>(i) = Point3f(rot(i, 0), rot(i, 1), rot(i, 2));
  }
  sbaR = sbaR.t();
  sbaTvec = -sbaR*sbaTvec;
  Rodrigues(sbaR, rvec);

  for(size_t i = 0; i < points.size(); i++)
  {
    points[i] = Point3f(sba0.tracks[i].point(0), sba0.tracks[i].point(1), sba0.tracks[i].point(2));
#if defined(_DEBUG_CONSOLE)
    printf("%f %f %f\n", points[i].x, points[i].y, points[i].z);
#endif
  }
}

float calcOptimalPointCloudScale(const vector<Point3f>& points1, const vector<Point3f>& points2)
{
  assert(points1.size() == points2.size());

  float sum1 = 0.0f, sum2 = 0.0f;
  for(size_t i = 0; i < points1.size(); i++)
  {
    sum1 += points1[i].dot(points2[i]);
    sum2 += points1[i].dot(points1[i]);
  }

  return sum1/sum2;
}

float calcScaledPointCloudDistance(const vector<Point3f>& points1, const vector<Point3f>& points2)
{
  assert(points1.size() == points2.size());

  float s = calcOptimalPointCloudScale(points1, points2);

  printf("Optimal scale = %f\n", s);
  float sum = 0.0f;
  for(size_t i = 0; i < points1.size(); i++)
  {
    Point3f d = points1[i]*s - points2[i];
    sum += d.dot(d);

    printf("Pair %d: (%f %f %f) vs (%f %f %f), dist = %f\n", (int)i, points1[i].x*s, points1[i].y*s,
           points1[i].z*s, points2[i].x, points2[i].y, points2[i].z, sqrt(d.dot(d)));
  }

  return sqrt(sum/points1.size());
}

void findNaNPoints(const vector<Point3f>& points, vector<bool>& valid)
{
  valid.resize(points.size());
  for(size_t i = 0; i < points.size(); i++)
  {
    if(cvIsNaN(points[i].x) || cvIsNaN(points[i].y) || cvIsNaN(points[i].z))
    {
      valid[i] = false;
    }
    else
    {
      valid[i] = true;
    }
  }
}

void calcRelativeRT(const Mat& R1, const Mat& T1, const Mat& R2, const Mat& T2, Mat& dR, Mat& dT)
{
  assert(R1.cols == R2.cols && R1.rows == R2.rows);
  Mat _R1, _R2;

  if(R1.cols == 1 || R1.rows == 1)
  {
    Rodrigues(R1, _R1);
    Rodrigues(R2, _R2);
  }
  else
  {
    _R1 = R1;
    _R2 = R2;
  }

  Mat R1inv = _R1.inv();
  Mat _dR = _R2*R1inv;

  dT = T2 - _R2*R1inv*T1;

  if(R1.cols == 1 || R1.rows == 1)
  {
    Rodrigues(_dR, dR);
  }
  else
  {
    dR = _dR;
  }
}

}
