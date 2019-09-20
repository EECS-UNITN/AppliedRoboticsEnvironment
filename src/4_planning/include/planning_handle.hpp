#pragma once

#include <ros/ros.h>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "planning/ComputePlan.h"
#include "utils.hpp"

namespace planning {


class PlanningHandle {

public:
  // Constructor
  explicit PlanningHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void subscribeToTopic(); 
  void initServices();
  
  // ROS node handle
  ros::NodeHandle nh_;
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  bool default_implementation_;
  std::string config_folder_;
  
  // ROS communication  
  ros::Subscriber sub_victims_, sub_obstacles_, sub_gate_, sub_robot_;
  ros::Publisher  pub_plan_, pub_dt_, pub_plan_rviz_;
  ros::ServiceServer srv_plan_;
  bool has_victims_, has_obstacles_, has_gate_, has_robot_;

  // TOPICS  
  std::string sub_victims_topic_name_, sub_obstacles_topic_name_, sub_gate_topic_name_, sub_robot_topic_name_;
  std::string pub_plan_topic_name_, pub_dt_topic_name_, pub_plan_rviz_topic_name_;
  std::string srv_plan_topic_name_;

  std::vector<Polygon> obstacle_list_;
  std::vector<std::pair<int,Polygon>> victim_list_;
  Polygon gate_, borders_;
  double x_, y_, theta_;
  Path path_;

  std_msgs::Header header_;

  // Callback
  void robotCb(const geometry_msgs::PoseStampedPtr robot_pose);
  void victimsCb(const jsk_recognition_msgs::PolygonArrayPtr victims);
  void obstaclesCb(const jsk_recognition_msgs::PolygonArrayPtr obstacles);
  void gateCb(const jsk_recognition_msgs::PolygonArrayPtr gate);

  // Service
  bool computePlanSrv(ComputePlan::Request& req, ComputePlan::Response& res);

};

}
