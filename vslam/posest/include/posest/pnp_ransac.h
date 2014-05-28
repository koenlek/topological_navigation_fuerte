#if !defined(_PNP_RANSAC_H)
#define _PNP_RANSAC_H

#include "cv.h"

#define MIN_POINTS_COUNT 4

void project3dPoints(const std::vector<cv::Point3f>& points, const cv::Mat& rvec, const cv::Mat& tvec,
                     std::vector<cv::Point3f>& modif_points);

bool solvePnPRansac(const std::vector<cv::Point3f>& object_points, const std::vector<cv::Point2f>& image_points, const cv::Mat& camera_matrix, const cv::Mat& distCoeffs,
		  cv::Mat& rvec, cv::Mat& tvec, bool use_extrinsic_guess = false,  int num_iterations = 100,
		  float max_dist = 2.0, int min_inlier_num = -1, std::vector<int>* inliers = NULL);

#endif
