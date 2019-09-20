#pragma once

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#endif

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include <vector>

#include "utilsIP.hpp"

namespace image_proc {

class GrabFrameHandle {

 public:
  // Constructor
  explicit GrabFrameHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void connectCb(); // when a subscruiber want the rectify img
  
  bool initialized_;
  // ROS node handle
  ros::NodeHandle nh_;
  int queue_size_;
  bool default_implementation_;
  std::string config_folder_;
  
  // ROS communication  
  ros::Publisher  pub_dt_,  pub_img_;  
  ros::Timer timer_;

  double freq_;
  boost::mutex connect_mutex_; // mutex to control subscriber number

  // TOPICS    
  std::string pub_img_topic_name_, pub_dt_topic_name_;

  // Callback
  void grabCb(const ros::TimerEvent& evt);

};
}