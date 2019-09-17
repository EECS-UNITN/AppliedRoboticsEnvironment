#include "professor_interface.hpp"

#include <stdexcept>
#include <sstream>
namespace professor {

  std::string initImageFolder(){
  	return "/home/valerio/Pictures/";
  }	

  void genericImageListener(const cv::Mat& img_in, std::string topic){
  	throw std::logic_error( "PROFESSOR FUNCTION NOT LOADED" );
  }

bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec){
   
    return false;
  }

	void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
					const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs){

    throw std::logic_error( "PROFESSOR FUNCTION NOT LOADED" );	

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, cv::Mat& plane_transf){
    throw std::logic_error( "PROFESSOR FUNCTION NOT LOADED" );  
  }


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const double scale){
    throw std::logic_error( "PROFESSOR FUNCTION NOT LOADED" );   
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate){
    throw std::logic_error( "PROFESSOR FUNCTION NOT LOADED" );   
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta){
    throw std::logic_error( "PROFESSOR FUNCTION NOT LOADED" );    
  }

}

