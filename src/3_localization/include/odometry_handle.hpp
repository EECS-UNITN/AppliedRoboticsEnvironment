#pragma once

#include <ros/ros.h>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Odometry.h"

#include "extended_kalman_filter.hpp"
#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace localization {

class OdometryHandle {

public:
  // Constructor
  explicit OdometryHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void subscribeToTopic(); 
  
  ExtendedKalmanFilter ekf_;

  // ROS node handle
  ros::NodeHandle nh_;
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  
  std::string frame_id_, child_frame_id_;
  geometry_msgs::TwistWithCovarianceStamped last_twist_;

  // ROS communication
  ros::Subscriber sub_twist_;
  ros::Publisher pub_odom_;
  ros::Timer pub_timer_;

  // TOPICS  
  std::string sub_twist_topic_name_;
  std::string pub_odom_topic_name_;
  
  // Robot state
  ros::Time stamp_;
  double x_, y_, yaw_, v_, yaw_r_;
  double integration_time_;
  bool speed_init_;

  // Callback
  void twistCb(geometry_msgs::TwistWithCovarianceStampedPtr twist);  
  void timerCallback(const ros::TimerEvent&);
};
}
