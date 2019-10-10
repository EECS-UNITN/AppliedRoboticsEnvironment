#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_elab/PlaneTransform.h>

#include <vector>
#include "utilsIP.hpp"

namespace image_proc {

class ObstacleDetectorHandle {

 public:
  // Constructor
  explicit ObstacleDetectorHandle();
    
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
  std::string config_folder_;
  
  // ROS communication  
  ros::Subscriber sub_image_, sub_transf_;
  ros::Publisher  pub_obstacles_, pub_gate_, pub_victims_, pub_dt_, pub_perimeter_, pub_victims_number_;
  // TOPICS 
  std::string frame_id_;
  std::string sub_image_topic_name_, sub_transf_topic_name_;
  std::string pub_obstacles_topic_name_, pub_gate_topic_name_, pub_victims_topic_name_, pub_dt_topic_name_, pub_perimeter_topic_name_, pub_victims_number_topic_name_;
  
  // CALIBRATION MATRIX  
  bool has_transform_;
  cv::Mat transform_;  
  double scale_;
  double arena_w_, arena_h_;

  std::vector<Polygon> obstacle_list_;
  std::vector<std::pair<int,Polygon>> victim_list_;
  Polygon gate_;
  

  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void transformCb(const image_elab::PlaneTransform& transf);
};
}
