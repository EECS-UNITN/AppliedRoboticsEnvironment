#pragma once

#include <opencv2/opencv.hpp>

namespace student {

void genericImageListener(const cv::Mat& img_in, std::string topic);


void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
				const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs);


}
