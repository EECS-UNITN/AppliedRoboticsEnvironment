#include "odometry_handle.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <assert.h>
#include <sstream>


const std::string kPringName = "odometry_handle.hpp";

namespace localization {

    // Constructor
    OdometryHandle::OdometryHandle():x_(0), y_(0), yaw_(0), v_(0), yaw_r_(0), speed_init_(false){
        ROS_DEBUG_NAMED(kPringName, "Constructor");        
    }

    void OdometryHandle::onInit(ros::NodeHandle &nodeHandle){
        nh_ = nodeHandle;
        initialized_ = true;

        loadParameters();    
        publishToTopics();
        subscribeToTopic();
    }

    template <class T>
    void loadVariable(ros::NodeHandle &nh, std::string variable_name, T* ret_val){
        T val;    
        if (!nh.getParam(variable_name, *ret_val)) 
        {          
            std::stringstream ss;
            
            ROS_ERROR_STREAM("Did not load " << variable_name);
            throw std::logic_error("Did not load " + variable_name);
        }
    }


    // Methods
    void OdometryHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");
      
        queue_size_ = 1;
        x_ = y_ = yaw_ = v_ = yaw_r_ = 0;
        speed_init_ = false;
        sub_twist_topic_name_ = "/sensor/twist";
        pub_odom_topic_name_  = "/estimation/odom";  

        integration_time_ = 0.005;
        frame_id_ = "odom";      

    }

    void OdometryHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        pub_odom_ = nh_.advertise<nav_msgs::Odometry>(pub_odom_topic_name_, 1, true);

    }

    void OdometryHandle::subscribeToTopic() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
        
        pub_timer_ = nh_.createTimer(ros::Duration(integration_time_), &OdometryHandle::timerCallback, this);
        sub_twist_ = nh_.subscribe(sub_twist_topic_name_, 1 , &OdometryHandle::twistCb, this, ros::TransportHints().tcpNoDelay());
    }

    void OdometryHandle::timerCallback(const ros::TimerEvent&){
        if(speed_init_){
            const double dt = integration_time_; //(ros::Time::now() - stamp_).toSec();
            const double ds  = v_*dt;
            const double dth = yaw_r_*dt;

            x_   += ds*std::cos(yaw_ + dth/2.);
            y_   += ds*std::sin(yaw_ + dth/2.);
            yaw_ += dth;

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_);

            nav_msgs::Odometry odom;
            odom.header.stamp    = ros::Time::now(); //twist->header.stamp;
            odom.header.frame_id = frame_id_;
            odom.child_frame_id  = last_twist_.header.frame_id;

            odom.twist = last_twist_.twist;
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.position.z = 0;            
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();
            pub_odom_.publish(odom);

        }
    }

    void OdometryHandle::twistCb(geometry_msgs::TwistWithCovarianceStampedPtr twist){    
        stamp_ = ros::Time::now(); //twist->header.stamp;
        v_     = twist->twist.twist.linear.x;
        yaw_r_ = twist->twist.twist.angular.z;   
        child_frame_id_ = twist->header.frame_id;
        last_twist_ = *twist;   
        speed_init_ = true;  
    }
}