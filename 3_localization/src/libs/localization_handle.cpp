#include "localization_handle.hpp"
#include "std_msgs/Float32.h" // used to publish dt

#include "tf/transform_broadcaster.h"

#include <assert.h>
#include <sstream>


const std::string kPringName = "localization_handle.hpp";

namespace localization {

    // Constructor
    LocalizationHandle::LocalizationHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_   = false;  
        has_gps_       = false;
        has_odom_      = false;
    }

    void LocalizationHandle::onInit(ros::NodeHandle &nodeHandle){
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
    void LocalizationHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");
      
        queue_size_ = 1;

        sub_gps_topic_name_      = "/estimation/pose";
        sub_odom_topic_name_     = "/estimation/odom";
        pub_map_odom_topic_name_ = "/estimation/map";
        pub_dt_topic_name_       = "/process_time/localization";

    }

    void LocalizationHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        pub_map_odom_ = nh_.advertise<nav_msgs::Odometry>(pub_map_odom_topic_name_, 1, true);
      
        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, true);
    }

    void LocalizationHandle::subscribeToTopic() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
        
        sub_odom_ = nh_.subscribe(sub_odom_topic_name_, queue_size_, &LocalizationHandle::odomCb, this);

        sub_gps_ = nh_.subscribe(sub_gps_topic_name_, queue_size_, &LocalizationHandle::gpsCb, this);
    }


    void LocalizationHandle::gpsCb(const geometry_msgs::PoseStampedPtr robot_pose){

        if(odom_list_.size() < 2){
            ROS_WARN_STREAM("Localization handle waiting more odom msg");
            return;
        }

        //EKF PREDICTION
        const auto& gps_stamp = robot_pose->header.stamp;
        bool found = false;
        if(odom_list_.front().stamp < gps_stamp && odom_list_.back().stamp > gps_stamp){
            std::list<RobotState>::iterator odo_el;
            while(odom_list_.size()>2 && !found){
                odo_el = odom_list_.begin();
                std::advance(odo_el,1);
                if(odo_el->stamp < gps_stamp){
                    found = true;
                }
                // Predict
                const RobotState& rhs(odom_list_.front());
                const RobotState& lhs(*odo_el);
                const double ds = std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
                double dth = lhs.theta - rhs.theta;

                while(dth > M_PI)  dth-=2*M_PI;
                while(dth < -M_PI) dth+=2*M_PI;
                

                const double ds_std_scale  = 0.05;
                const double dth_std_scale = 0.05;
                const double ds_std  = ds_std_scale  * ds;
                const double dth_std = dth_std_scale * dth;

                const double ds_cov  = ds_std*ds_std;
                const double dth_cov = dth_std*dth_std;
                Vec2 u;
                Matrix2 Q;
                u << ds, dth;

                Q << ds_cov,       0,
                          0, dth_cov;
                  

                ekf_.predict(u, Q);
                odom_list_.pop_front();
            }    
        }else{
            if(odom_list_.back().stamp < gps_stamp){
                ROS_WARN_STREAM("Localization handle gps msg is in the future ");
                auto last_el = odom_list_.end();
                last_el--;
                odom_list_.erase(odom_list_.begin(), last_el);                
            }else{
                ROS_WARN_STREAM("Localization handle no odometry msg with timestamp corresponding to gps msg");
            }
            ekf_.reset();
        }
        

        {
        // EKF UPDATE
        const double& x = robot_pose->pose.position.x;
        const double& y = robot_pose->pose.position.y;
        const tf::Quaternion q(robot_pose->pose.orientation.x, 
            robot_pose->pose.orientation.y, 
            robot_pose->pose.orientation.z, 
            robot_pose->pose.orientation.w);
        const tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);


        const double xy_cov = 0.002*0.002;
        const double th_cov = 0.005*0.005;
        Vec3 z;
        Matrix3 R;
        z << x, y, yaw;

        R << xy_cov,      0,      0, 
                  0, xy_cov,      0, 
                  0,      0, th_cov;

        ekf_.updateGPS(z, R);
        }
        

        // COMPUTE TRANSFORM         
        const RobotState& odom(odom_list_.front());

        const double dth = odom.theta - ekf_.x(2); // theta odom - theta map
        Vec2 p_odo, p_map;
        p_odo << odom.x, odom.y;
        p_map << ekf_.x(0), ekf_.x(1);

        const double cth = std::cos(dth);
        const double sth = std::sin(dth);
        R_ << cth, -sth,
             sth,  cth;   

        t_ = p_map - R_*p_odo;

        has_gps_ = true;
    }

  void LocalizationHandle::odomCb(const nav_msgs::OdometryPtr odom){

    // STORE ODOM MSG
    const double x = odom->pose.pose.position.x;
    const double y = odom->pose.pose.position.y;
    const tf::Quaternion q(odom->pose.pose.orientation.x, 
        odom->pose.pose.orientation.y, 
        odom->pose.pose.orientation.z, 
        odom->pose.pose.orientation.w);
    const tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(odom_list_.empty() || odom->header.stamp > odom_list_.back().stamp){
        odom_list_.emplace_back(odom->header.stamp, x, y, yaw);
    }
    // Publish last odom rotated to map
    if(has_gps_){
        //
        Vec2 p_odo, p_map;
        p_odo << x, y;

        p_map = R_ * p_odo + t_;

        const double dth = std::atan2(R_(0,1), R_(0,0));

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw - dth);

        // Rototranslate odometry
        nav_msgs::OdometryPtr map(new nav_msgs::Odometry);
        map->header = odom->header;
        map->pose.pose.position.x = p_map(0);
        map->pose.pose.position.y = p_map(1);
        map->pose.pose.position.z = 0;
        map->pose.pose.orientation = odom_quat;
        pub_map_odom_.publish(map);
    }
  }
}