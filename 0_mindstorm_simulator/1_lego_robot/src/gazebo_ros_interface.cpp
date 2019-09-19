// Main Include
#include "gazebo_ros_interface.hpp"
#include "tf/transform_broadcaster.h"
// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

namespace gazebo {

std::pair<double, double> gaussiaNoise2D(const double& mu1, const double& sigma1, const double& mu2, const double& sigma2) {
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables
    // see wikipedia
    double U = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    double V = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    double Y = sqrt(-2.0 * ::log(U)) * sin(2.0 * M_PI * V); // the other indep. normal variable
    // we'll just use X
    // scale to our mu and sigma
    X = sigma1 * X + mu1;
    Y = sigma2 * Y + mu2;
    return std::pair<double,double>(X, Y);
}

LegoModelPlugin::LegoModelPlugin():
    x_car_(0), y_car_(0),  yaw_car_(0), v_car_(0), yaw_r_car_(0)
{
    int  argc  = 0;
    char *argv = nullptr;
    ros::init(argc, &argv, "LegoModelPlugin");
    update_from_last_cmd_ = 0;

    measure_alpha_v_ = measure_alpha_yaw_r_ = 0.01;     // 1% noise
    actuation_alpha_v_ = actuation_alpha_yaw_r_ = 0.01; // 1% noise
    
    nh_ = ros::NodeHandle();
    sub_set_pose_ = nh_.subscribe("/initialpose", 1, &LegoModelPlugin::initialPoseCb, this);
    sub_twist_ = nh_.subscribe("/control/cmd_vel",1, &LegoModelPlugin::setTwistCb, this);

    pub_twist_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/sensor/odom", 1, true);

    pub_ideal_odom_ = nh_.advertise<nav_msgs::Odometry>("/ideal/odom", 1, true);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&LegoModelPlugin::onUpdate, this));

}

void LegoModelPlugin::setTwistCb(geometry_msgs::TwistConstPtr twist){    
    // TODO: add noise
    update_from_last_cmd_ = 0;
    const double v_std = actuation_alpha_v_* twist->linear.x;
    const double yr_std =  actuation_alpha_yaw_r_ * twist->angular.z;
    std::tie(v_car_,yaw_r_car_) = gaussiaNoise2D(twist->linear.x, v_std, twist->angular.z, yr_std);
}

void LegoModelPlugin::initialPoseCb(geometry_msgs::PoseWithCovarianceStampedConstPtr pose){    

    if(pose->header.frame_id.compare("map")){
        return;
    }
    x_car_ = pose->pose.pose.position.x;
    y_car_ = pose->pose.pose.position.y;
    const tf::Quaternion q(pose->pose.pose.orientation.x, 
            pose->pose.pose.orientation.y, 
            pose->pose.pose.orientation.z, 
            pose->pose.pose.orientation.w);
    const tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_car_);

    v_car_ = 0;
    yaw_r_car_ = 0;

    setModelState();
}

LegoModelPlugin::~LegoModelPlugin() 
{
    
}

void LegoModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
{
    gzmsg << "Loading LegoModelPlugin" << std::endl;
    gzmsg << "LegoModelPlugin loading params" << std::endl;

    model_ = _model;
    world_ = model_->GetWorld();
}

void LegoModelPlugin::Reset() 
{
    update_from_last_cmd_ = 0;
    last_sim_time_ = 0;
    x_car_     = 0;
    y_car_     = 0;
    yaw_car_   = 0;
    v_car_     = 0;
    yaw_r_car_ = 0;
}

void LegoModelPlugin::onUpdate() 
{   
    static bool msg_sent = false; 
    std::lock_guard<std::mutex> lock(mtx_);
    common::Time curTime = world_->GetSimTime();
    double dt = 0.0;
    if (!isLoopTime(curTime, dt)) {
        return;
    }
       
    if(update_from_last_cmd_ > 20){        
        if(!msg_sent){
            gzwarn << "Setting speed to 0 not reciving cmd" << std::endl;
            msg_sent = true;
        }
        // Stop the car if reciving no cmd 
        v_car_   = 0;
        yaw_r_car_ = 0;
    }else{
        msg_sent = false;
    }
    
    // FIX if yaw_r_car_ > 0 --> approx as a circ arc
    const double dth = yaw_r_car_ * dt;
    const double c = std::cos(yaw_car_ + dth/2);
    const double s = std::sin(yaw_car_ + dth/2);
    x_car_   += c*v_car_ * dt;   
    y_car_   += s*v_car_ * dt;
    yaw_car_ += dth;

    setModelState();    
    update_from_last_cmd_++;



}
void LegoModelPlugin::publishInfo(){
    // IDEAL ODOMETRY
    nav_msgs::OdometryPtr map(new nav_msgs::Odometry);
    map->header.frame_id   = "map";
    map->header.stamp.sec  = last_sim_time_.sec;
    map->header.stamp.nsec = last_sim_time_.nsec;
    map->pose.pose.position.x = x_car_;
    map->pose.pose.position.y = yaw_car_;
    map->pose.pose.position.z = 0;
    const geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_car_);
    map->pose.pose.orientation = odom_quat;
    pub_ideal_odom_.publish(map);


    // ACTUAL SPEED
    
    geometry_msgs::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.frame_id   = "robot_footprint";
    twist_msg.header.stamp.sec  = last_sim_time_.sec;    
    twist_msg.header.stamp.nsec = last_sim_time_.nsec;

    // TODO: add noise to the measure
    double measure_std_v =  measure_alpha_v_ * v_car_;
    double measure_std_yr = measure_alpha_yaw_r_ * yaw_r_car_;
    double measured_v, measured_yr;
    std::tie(measured_v, measured_yr) = gaussiaNoise2D(v_car_, measure_std_v, yaw_r_car_, measure_std_yr);

    twist_msg.twist.twist.linear.x = measured_v;
    twist_msg.twist.twist.angular.z = measured_yr;
    twist_msg.twist.covariance[0]  = measure_std_v * measure_std_v;
    twist_msg.twist.covariance[35] = measure_std_yr * measure_std_yr;

    // No correlation betwheen measure
    twist_msg.twist.covariance[5]  = 0;
    twist_msg.twist.covariance[30]  = 0;
    pub_twist_.publish(twist_msg);
}

bool LegoModelPlugin::isLoopTime(const common::Time &time, double &dt) 
{
    dt = (time - last_sim_time_).Double();
    if (dt < 0.0) {
        this->Reset();
        return false;
    } else if (ignition::math::equal(dt, 0.0)) {
        return false;
    }
    last_sim_time_ = time;
    return true;
}

void LegoModelPlugin::setModelState() 
{    
    const math::Pose    pose(x_car_, y_car_, 0.0, 0, 0.0, yaw_car_);
    const math::Vector3 vel(v_car_, 0, 0.0);
    const math::Vector3 angular(0.0, 0.0, yaw_r_car_);
    model_->SetWorldPose(pose);
    model_->SetAngularVel(angular);
    model_->SetLinearVel(vel);
}

GZ_REGISTER_MODEL_PLUGIN(LegoModelPlugin)
} // namespace gazebo


