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
      
        odom_list_.clear();
        queue_size_ = 1;

        loadVariable<double>(nh_,"/arena/w",&arena_w_);
        loadVariable<double>(nh_,"/arena/h",&arena_h_);

        sub_gps_topic_name_      = "/estimation/pose";
        sub_odom_topic_name_     = "/estimation/odom"; //"/ideal/odom"; 
        pub_map_odom_topic_name_ = "/estimation/map";
        pub_dt_topic_name_       = "/process_time/localization";

        has_new_gps_ = false;

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
        
        sub_odom_ = nh_.subscribe(sub_odom_topic_name_, queue_size_, &LocalizationHandle::odomCb, this, ros::TransportHints().tcpNoDelay());

        sub_gps_ = nh_.subscribe(sub_gps_topic_name_, queue_size_, &LocalizationHandle::gpsCb, this, ros::TransportHints().tcpNoDelay());
    }


    void LocalizationHandle::gpsCb(const geometry_msgs::PoseStampedPtr robot_pose){
        ROS_DEBUG_NAMED(kPringName, "gpsCb");
        const double arena_margin = 0.15;
                
        if(odom_list_.size() < 2){
            ROS_WARN_STREAM("Localization handle waiting more odom msg");
            return;
        }

        // Extract the measure and check if they are not outlayers
        frame_id_ = robot_pose->header.frame_id;
        gps_stamp_ = robot_pose->header.stamp;        
        gps_x_ = robot_pose->pose.position.x;
        gps_y_ = robot_pose->pose.position.y;
        const tf::Quaternion gps_q(robot_pose->pose.orientation.x, 
            robot_pose->pose.orientation.y, 
            robot_pose->pose.orientation.z, 
            robot_pose->pose.orientation.w);
        const tf::Matrix3x3 gps_m(gps_q);
        double gps_roll, gps_pitch;
        gps_m.getRPY(gps_roll, gps_pitch, gps_yaw_);

        // Raugh ceck on the measure! Must be inside the arena
        if(!std::isfinite(gps_x_) || !std::isfinite(gps_y_) || !std::isfinite(gps_yaw_) 
            || gps_x_ < -arena_margin || gps_x_ > (arena_w_ + arena_margin) 
            || gps_y_ < -arena_margin || gps_y_ > (arena_h_ + arena_margin)){
            ROS_WARN_STREAM("UPS! Discarding the measure (position out of the arena!)");
            return;
        }

        has_new_gps_ = true;        

    }

    void LocalizationHandle::updateFilter(){
        const double xy_max_err   = 0.1;
        const double yaw_max_err  = M_PI/180*10;

        if(!has_new_gps_) return;
        has_new_gps_ = false;
        bool found = false; 
        RobotState removed_state;  // Last removed odom state!          

        if(ekf_.isLocalized()){            
            //EKF PREDICTION                        
            if(odom_list_.front().stamp <= gps_stamp_ && odom_list_.back().stamp > gps_stamp_){
                std::list<RobotState>::iterator odo_el;                       
                while(odom_list_.size()>2 && !found){
                    odo_el = odom_list_.begin();
                    std::advance(odo_el,1);
                    if(odo_el->stamp > gps_stamp_){
                        found = true;
                    }
                    // Predict
                    const RobotState& rhs(odom_list_.front());
                    const RobotState& lhs(*odo_el);
                    const double ds = std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
                    double dth = lhs.theta - rhs.theta;

                    while(dth > M_PI)  dth-=2*M_PI;
                    while(dth < -M_PI) dth+=2*M_PI;
                    

                    const double ds_std_scale  = 0.1;
                    const double dth_std_scale = 0.1;
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
                    
                    if(found){
                        removed_state = odom_list_.front();
                    }

                    odom_list_.pop_front();                     
                }    
            }else{            
                if(odom_list_.back().stamp < gps_stamp_){
                    ROS_WARN_STREAM("Localization handle gps msg is in the future ");
                    auto last_el = odom_list_.end();
                    last_el--;
                    odom_list_.erase(odom_list_.begin(), last_el);               
                }else{
                    ROS_WARN_STREAM("Localization handle no odometry msg with timestamp corresponding to gps msg");
                    return;                 
                }

                ekf_.reset();
            }
            
            // CHECK IF PREDICTION AND MESURE DO AGREE
            const double delta_x = gps_x_ - ekf_.x(0);
            const double delta_y = gps_y_ - ekf_.x(1);
            double delta_a = gps_yaw_ - ekf_.x(2);
            
            while(delta_a < - M_PI) delta_a+=2*M_PI;        
            while(delta_a >   M_PI) delta_a-=2*M_PI;
            
            // TODO: better to compute this using the covariance of prediction!
            const double dist_err = std::hypot(delta_x, delta_y);
            if(dist_err > xy_max_err || std::abs(delta_a) > yaw_max_err ){
                ROS_WARN_STREAM("SETTING TO MEASURE: serr, therr" << dist_err <<", " << delta_a);
                ekf_.reset();
            }

        } // IF EKF LOCALIZED
             
        bool ekf_ok = ekf_.isLocalized();
        {
            // EKF UPDATE                
            const double xy_cov = 0.01*0.01;
            const double th_cov = 0.05*0.05;
            Vec3 z;
            Matrix3 R;
            z << gps_x_, gps_y_, gps_yaw_;

            R << xy_cov,      0,      0, 
                      0, xy_cov,      0, 
                      0,      0, th_cov;

            ekf_.updateGPS(z, R);
        }
        

        if(ekf_ok && found){
            // Interpolate x,y, theta
            const double DT = (odom_list_.front().stamp - removed_state.stamp).toSec();
            const double dt = (gps_stamp_ - removed_state.stamp).toSec(); 
            const double scale = dt/DT;

            const double dx = (odom_list_.front().x - removed_state.x)*scale;
            const double dy = (odom_list_.front().y - removed_state.y)*scale;

            double yaw_step = odom_list_.front().theta - removed_state.theta;
            while(yaw_step < -M_PI ) yaw_step += 2*M_PI;
            while(yaw_step >  M_PI ) yaw_step -= 2*M_PI;
            
            const double dyaw = yaw_step*scale;

            const double x_i   = removed_state.x + dx;        
            const double y_i   = removed_state.y + dy;
            const double yaw_i = removed_state.theta + dyaw;

            // COMPUTE TRANSFORM         
            const double dth = yaw_i - ekf_.x(2); // theta odom - theta map
            Vec2 p_odo, p_map;
            p_odo << x_i, y_i;
            p_map << ekf_.x(0), ekf_.x(1);

            const double cth = std::cos(dth);
            const double sth = std::sin(dth);
            R_ << cth, -sth,
                  sth,  cth;   

            t_ = p_map - R_*p_odo;

            has_gps_ = true;            
        }
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

        while(odom_list_.size() > 200){
            odom_list_.pop_front();
        }
    }

    // Try to update the filter
    updateFilter();
    

    // Publish last odom rotated to map
    if(has_gps_){
        //
        Vec2 p_odo, p_map;
        p_odo << x, y;

        p_map = R_ * p_odo + t_;

        const double dth = std::atan2(R_(1,0), R_(0,0));
        if(!std::isfinite(p_map(0)) || !std::isfinite(p_map(1)) || 
            !std::isfinite(dth)){
            return;
        }

        // Rototranslate odometry
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw - dth);
        nav_msgs::OdometryPtr map(new nav_msgs::Odometry);
        map->header.stamp = odom->header.stamp;
        map->header.frame_id = frame_id_;
        map->pose.pose.position.x = p_map(0);
        map->pose.pose.position.y = p_map(1);
        map->pose.pose.position.z = 0;
        map->pose.pose.orientation = odom_quat;
        pub_map_odom_.publish(map);


        // // SHOW EKF STATE
        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(ekf_.x(2));

        // // Rototranslate odometry
        // nav_msgs::OdometryPtr map(new nav_msgs::Odometry);
        // map->header.stamp = odom->header.stamp;
        // map->header.frame_id = frame_id_;
        // map->pose.pose.position.x = ekf_.x(0);
        // map->pose.pose.position.y = ekf_.x(1);
        // map->pose.pose.position.z = 0;
        // map->pose.pose.orientation = odom_quat;
        // pub_map_odom_.publish(map);
    }
  }
}