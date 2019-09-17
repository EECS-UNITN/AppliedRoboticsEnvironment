#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include <vector>

#include "utils.hpp"

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
  ros::Publisher  pub_ground_transf_, pub_robot_transf_;

  // TOPICS  
  std::string rect_subscriber_topic_name_;
  std::string extr_publisher_topic_name_, pub_ground_topic_name_, pub_robot_topic_name_;

  // CALIBRATION MATRIX
  cv::Mat camera_matrix_;
  cv::Mat tvec_, rvec_;
  cv::Mat ground_plane_, robot_plane_;
  bool calib_done_;
  double scale_;

  std::vector<cv::Point3f> object_points_ground_, object_points_robot_;

  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

};
}