#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_elab/PlaneTransform.h>

#include <vector>

#include "utils.hpp"

namespace image_proc {

class RobotDetectorHandle {

 public:
  // Constructor
  explicit RobotDetectorHandle();
    
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
  ros::Subscriber sub_image_, sub_transf_;
  ros::Publisher  pub_robot_;
  ros::Publisher  pub_gps_loc_;
  
  // TOPICS  
  std::string sub_image_topic_name_, sub_transf_topic_name_;
  std::string pub_robot_topic_name_, pub_gps_loc_topic_name_;

  // CALIBRATION MATRIX  
  bool has_transform_;
  cv::Mat transform_;  
  double scale_;

  Polygon triangle_;
  double x_, y_, theta_;

  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void transformCb(const image_elab::PlaneTransform& transf);
};
}
