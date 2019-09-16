#include "extrinsic_handle.hpp"
#include "student_interface.hpp"

#include "image_elab/ExtrinsicParams.h"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
using namespace image_proc;

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "rectify_handle.hpp";

// Constructor
ExtrinsicHandle::ExtrinsicHandle(){
    ROS_INFO_NAMED(kPringName, "Constructor");
    initialized_  = false;    
}

void ExtrinsicHandle::onInit(ros::NodeHandle &nodeHandle){
    nh_ = nodeHandle;
    initialized_ = true;

    loadParameters();    
    publishToTopics();
    subscribeToTopic();
}

template <class T>
void loadVariable(ros::NodeHandle &nh, std::string variable_name, T* ret_val){
    T val;    
    if (!nh.getParam(variable_name, *ret_val)) 
    {          
        std::stringstream ss;
        
        ROS_ERROR_STREAM("Did not load " << variable_name);
        throw std::logic_error("Did not load " + variable_name);
    }
}


// Methods
void ExtrinsicHandle::loadParameters() {
    ROS_INFO_NAMED(kPringName, "Loading Params");
  
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    
    loadVariable<double>(nh_, "/camera_calibration/fx", &fx);
    loadVariable<double>(nh_, "/camera_calibration/fy", &fy);
    loadVariable<double>(nh_, "/camera_calibration/cx", &cx);
    loadVariable<double>(nh_, "/camera_calibration/cy", &cy);
    loadVariable<double>(nh_, "/camera_calibration/k1", &k1);
    loadVariable<double>(nh_, "/camera_calibration/k2", &k2);
    loadVariable<double>(nh_, "/camera_calibration/k3", &k3);

    loadVariable<double>(nh_, "/camera_calibration/p1", &p1);
    loadVariable<double>(nh_, "/camera_calibration/p2", &p2);

    loadVariable<bool>(nh_,"/default_implementation/extrinsic_calib", &default_implementation_);
    queue_size_ = 1;
    extr_publisher_topic_name_   = "/image/extrinsic_calib";
    rect_subscriber_topic_name_  = "/camera/rgb/image_raw"; //"/image/rectify";
    //camera_subscriber_topic_name_  = "/camera/rgb/image_raw";

    calib_done_ = false;
    float w = 1.5f, h = 1.0f;
    object_points_ = {{0.f,0.f,0.f},{w,0.f,0.f},{w,h,0.f},{0.f,h,0.f}};


    dist_coeffs_   = (cv::Mat1d(1,4) << k1, k2, p1, p2, k3);
    camera_matrix_ = (cv::Mat1d(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    ROS_INFO_STREAM_NAMED(kPringName, "cametra_matrix = \n " << camera_matrix_);
    ROS_INFO_STREAM_NAMED(kPringName, "dist_coeffs = \n"    << dist_coeffs_ );
}

void ExtrinsicHandle::publishToTopics() {
    ROS_INFO_NAMED(kPringName, "Init publishers");
    assert (initialized_);


    // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
    pub_extr_  = nh_.advertise<image_elab::ExtrinsicParams>(extr_publisher_topic_name_, 1, true);
}

void ExtrinsicHandle::subscribeToTopic() {
    ROS_INFO_NAMED(kPringName, "Init subscribers");
    assert (initialized_);
     
    sub_rect_ = nh_.subscribe(rect_subscriber_topic_name_, queue_size_, &ExtrinsicHandle::imageCb, this);

}



#include <vector>
#include <atomic>
#include <unistd.h>

// Defintion of the function pickNPoints and the callback mouseCallback.
// The function pickNPoints is used to display a window with a background
// image, and to prompt the user to select n points on this image.
cv::Mat bg_img;
std::vector<cv::Point2f> result;
std::string name;
std::atomic<bool> done;
int n;

void mouseCallback(int event, int x, int y, int, void* p)
{
  if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;
  
  result.emplace_back(x, y);
  cv::circle(bg_img, result.back(), 20, cv::Scalar(0,0,255), -1);
  cv::imshow(name.c_str(), bg_img);

  if (result.size() >= n) {
    usleep(500*1000);
    done.store(true);
  }
}

std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
{
  result.clear();
  bg_img = img.clone();
  name = "Pick " + std::to_string(n0) + " points";
  cv::imshow(name.c_str(), bg_img);
  cv::namedWindow(name.c_str());
  n = n0;

  done.store(false);

  cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
  while (!done.load()) {
    cv::waitKey(500);
  }

  cv::destroyWindow(name.c_str());
  return result;
}





void ExtrinsicHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){    
    
    if (calib_done_) return;    

    // Convert to Opencs
    //cv_bridge::CvImagePtr cv_ptr; // use cv_bridge::toCvCopy with this 
    cv_bridge::CvImageConstPtr cv_ptr; 
    try
    {
        if (enc::isColor(msg->encoding))
          cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else
          cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv_bridge::CvImagePtr out_img(new cv_bridge::CvImage());

    try{
        if(default_implementation_){
            ROS_INFO_NAMED(kPringName, "Call default function");
            
            std::vector<cv::Point2f> image_points = pickNPoints(4, cv_ptr->image);

            std::cout << "object_points_: " << object_points_.size() << std::endl;
            for(auto pt:object_points_){
              std::cout << pt << std::endl;
            }
            std::cout << "image_points: " << image_points.size() << std::endl;
            for(auto pt:image_points){
              std::cout << pt << std::endl;
            }

            std::cerr << "RVEC: " << std::endl;
              for (int i=0; i<rvec_.rows; ++i) {
                for (int j=0; j<rvec_.cols; ++j) {
                  std::cerr << std::setw(3) << rvec_.at<float>(i,j) << " ";
                }      
                std::cerr << std::endl;
              }

              std::cerr << "TVEC: " << std::endl;
              for (int i=0; i<tvec_.rows; ++i) {
                for (int j=0; j<tvec_.cols; ++j) {
                  std::cerr << std::setw(3) << tvec_.at<float>(i,j) << " ";
                }      
                std::cerr << std::endl;
              }
            bool ok = cv::solvePnP(object_points_, image_points, camera_matrix_, dist_coeffs_, rvec_, tvec_);

            // cv::Mat Rt;
            // cv::Rodrigues(rvec_, Rt);
            // auto R = Rt.t();
            // auto pos = -R * tvec_;

            if (!ok) {
              std::cerr << "FAILED SOLVE_PNP" << std::endl;
            }
            else {
              image_elab::ExtrinsicParamsPtr extr_msg(new image_elab::  ExtrinsicParams());

              // std::cerr << "pos: " << pos << std::endl;
              // for (int i=0; i<pos.rows; ++i) {
              //   for (int j=0; j<pos.cols; ++j) {
              //     std::cerr << std::setw(3) << pos.at<float>(i,j) << " ";
              //   }      
              //   std::cerr << std::endl;
              // }

              std::cerr << "RVEC: " << std::endl;
              for (int i=0; i<rvec_.rows; ++i) {
                for (int j=0; j<rvec_.cols; ++j) {
                  std::cerr << std::setw(3) << rvec_.at<double>(i,j) << " ";
                  extr_msg->rvec.push_back(rvec_.at<double>(i,j));
                }      
                std::cerr << std::endl;
              }

              std::cerr << "TVEC: " << std::endl;
              for (int i=0; i<tvec_.rows; ++i) {
                for (int j=0; j<tvec_.cols; ++j) {
                  std::cerr << std::setw(3) << tvec_.at<double>(i,j) << " ";
                  extr_msg->tvec.push_back(tvec_.at<double>(i,j));
                }      
                std::cerr << std::endl;
              }

              pub_extr_.publish(extr_msg);

              calib_done_ = true;
              sub_rect_.shutdown();
            }
        }else{
            // CALL STUDENT FUNCTION    
            ROS_WARN_NAMED(kPringName, "Call student function");
            //student::extrinsicCalib(cv_ptr->image, object_points_, camera_matrix_, dist_coeffs_, rvec_, tvec_);
            std::exit(0);
        }
        //TODO: publish position of the camera in tf:TREE

        //TODO: use camera matrix to project up the points!!!!
    }catch(...){

    }
    
}