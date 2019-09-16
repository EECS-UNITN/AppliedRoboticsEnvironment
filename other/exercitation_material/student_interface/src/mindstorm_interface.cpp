#include "mindstorm_interface.hpp"


namespace student {
	void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
					const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs){
    std::cout << "HELLO 123" << std::endl;
		cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
	}
}
