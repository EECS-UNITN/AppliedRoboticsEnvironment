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

class GenericHandle {

 public:
  // Constructor
  explicit GenericHandle();
    
  void onInit(ros::NodeHandle &nodeHandle, std::string default_implementation_field, std::string camera_subscriber_topic_name);

private:
  // Methods
  void loadParameters();  
  void connectCb();
  
  // ROS node handle
  ros::NodeHandle nh_;
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  bool default_implementation_;

  // ROS communication  
  ros::Subscriber sub_camera_;
  
  // TOPICS  
  std::string camera_subscriber_topic_name_;
  std::string default_implementation_field_;
    
  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

};
}