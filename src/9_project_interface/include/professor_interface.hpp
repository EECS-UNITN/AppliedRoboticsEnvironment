#pragma once

#include <opencv2/opencv.hpp>

namespace professor {

void genericImageListener(const cv::Mat& img_in, std::string topic);

bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec);

void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
				const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs);

void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, cv::Mat& plane_transf);

void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const double scale);

}
