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
#include <sensor_msgs/Image.h>
#include <image_elab/PlaneTransform.h>

namespace image_proc {

class UnwarpHandle {

 public:
  // Constructor
  explicit UnwarpHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void subscribeToTopics();
  
  // ROS node handle
  ros::NodeHandle nh_;
  boost::mutex connect_mutex_; // mutex to control subscriber number
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  bool default_implementation_;
  std::string config_folder_;

  // ROS communication  
  ros::Subscriber sub_undistort_;
  ros::Subscriber sub_ground_; 
  ros::Subscriber sub_robot_;   
  ros::Publisher  pub_unwarp_robot_;
  ros::Publisher  pub_unwarp_ground_, pub_dt_;  


  // TOPICS  
  std::string sub_ground_topic_name_;
  std::string sub_robot_topic_name_;  
  std::string sub_undistort_topic_name_;
  std::string robot_publisher_topic_name_;
  std::string ground_publisher_topic_name_, pub_dt_topic_name_;
  

  cv::Mat ground_transf_, robot_transf_;
  double scale_robot_, scale_ground_;
  bool has_ground_transf_, has_robot_transf_;

  
  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void groundCb(const image_elab::PlaneTransform& transf);
  void robotCb(const image_elab::PlaneTransform& transf);

  void connectGroundCb();
  void connectRobotCb();

  int n_subscribers_ground_, n_subscribers_robot_;

};
}