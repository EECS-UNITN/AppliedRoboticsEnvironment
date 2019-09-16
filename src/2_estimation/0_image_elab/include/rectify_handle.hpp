#pragma once

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_proc/RectifyConfig.h>
#include <image_transport/image_transport.h>
#include <mutex>

namespace image_proc {

class RectifyHandle {

 public:
  // Constructor
  explicit RectifyHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void connectCb(); // when a subscruiber want the rectify img
  
  // ROS node handle
  ros::NodeHandle nh_;
  boost::mutex connect_mutex_; // mutex to control subscriber number
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  bool default_implementation_;

  // ROS communication  
  ros::Subscriber sub_camera_;
  ros::Publisher  pub_rect_;

  // TOPICS  
  std::string camera_subscriber_topic_name_;
  std::string rec_publisher_topic_name_;

  // CALIBRATION MATRIX
  cv::Mat dist_coeffs_, camera_matrix_;

  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

};
}