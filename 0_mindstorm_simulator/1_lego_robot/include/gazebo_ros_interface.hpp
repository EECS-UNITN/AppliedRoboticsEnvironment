#pragma once

// ROS Includes
#include <ros/ros.h>

// Gazebo Includes
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

namespace gazebo {

class LegoModelPlugin : public ModelPlugin {
 public:

    LegoModelPlugin();

    ~LegoModelPlugin() override;

    void Reset() override;

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

 private:

    void onUpdate();

    void publishInfo();

    bool isLoopTime(const common::Time &time, double &dt);

    void initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose);
    void setTwistCb(geometry_msgs::TwistConstPtr twist);

    void setModelState();


    
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_set_pose_, sub_twist_;
    ros::Publisher pub_ideal_odom_, pub_twist_;

    // GAZEBO
    event::ConnectionPtr updateConnection_; // Pointer to the update event connection
    physics::ModelPtr model_; // Pointer to the model
    physics::WorldPtr world_;    
    common::Time last_sim_time_;
    std::mutex mtx_;

    double x_car_, y_car_, yaw_car_, v_car_, yaw_r_car_;

    int update_from_last_cmd_;

    // Measure covariance
    double measure_alpha_v_, measure_alpha_yaw_r_;
    double actuation_alpha_v_, actuation_alpha_yaw_r_;

    std::string map_frame_id_;
};

} // namespace gazebo
