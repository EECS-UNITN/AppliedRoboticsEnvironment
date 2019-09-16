#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include <vector>

namespace image_proc {

class ExtrinsicHandle {

 public:
  // Constructor
  explicit ExtrinsicHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void subscribeToTopic(); // when a subscruiber want the rectify img
  
  // ROS node handle
  ros::NodeHandle nh_;
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  bool default_implementation_;

  // ROS communication  
  ros::Subscriber sub_rect_;
  ros::Publisher  pub_extr_;

  // TOPICS  
  std::string rect_subscriber_topic_name_;
  std::string extr_publisher_topic_name_;

  // CALIBRATION MATRIX
  cv::Mat dist_coeffs_, camera_matrix_;
  cv::Mat tvec_, rvec_;
  bool calib_done_;

  std::vector<cv::Point3f> object_points_;

  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

};
}