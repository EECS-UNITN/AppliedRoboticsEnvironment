#pragma once

#include <opencv2/opencv.hpp>
#include "utils.hpp"

namespace student {

void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder);

bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder);

void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
				const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder);

void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, cv::Mat& plane_transf, const std::string& config_folder);

void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const double scale, const std::string& config_folder);

bool processGtMap(std::string file_name, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, Polygon& perimeter);

bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder);

bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder);

}
