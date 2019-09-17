#include "rectify_handle.hpp"
#include "student_interface.hpp"
#include "professor_interface.hpp"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/Float32.h"

#include <sstream>
using namespace image_proc;

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "rectify_handle.hpp";

// Constructor
RectifyHandle::RectifyHandle(){
    ROS_INFO_NAMED(kPringName, "Constructor");
    initialized_  = false;     
}

void RectifyHandle::onInit(ros::NodeHandle &nodeHandle){
    nh_ = nodeHandle;
    initialized_ = true;

    loadParameters();    
    publishToTopics();
    connectCb();
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
void RectifyHandle::loadParameters() {
    ROS_INFO_NAMED(kPringName, "Loading Params");
  
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    
    loadVariable<double>(nh_, "/camera_calibration/fx", &fx);
    loadVariable<double>(nh_, "/camera_calibration/fy", &fy);
    loadVariable<double>(nh_, "/camera_calibration/cx", &cx);
    loadVariable<double>(nh_, "/camera_calibration/cy", &cy);
    loadVariable<double>(nh_, "/camera_calibration/k1", &k1);
    loadVariable<double>(nh_, "/camera_calibration/k2", &k2);
    loadVariable<double>(nh_, "/camera_calibration/k3", &k3);

    loadVariable<double>(nh_, "/camera_calibration/p1", &p1);
    loadVariable<double>(nh_, "/camera_calibration/p2", &p2);

    loadVariable<bool>(nh_,"/default_implementation/rectify",&default_implementation_);
    queue_size_ = 1;
    rec_publisher_topic_name_      = "/image/rectify";
    camera_subscriber_topic_name_  = "/camera/rgb/image_raw";
    pub_dt_topic_name_             = "/process_time/imageUndistort";


    dist_coeffs_   = (cv::Mat1d(1,4) << k1, k2, p1, p2, k3);
    camera_matrix_ = (cv::Mat1d(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    ROS_INFO_STREAM_NAMED(kPringName, "cametra_matrix = \n " << camera_matrix_);
    ROS_INFO_STREAM_NAMED(kPringName, "dist_coeffs = \n"    << dist_coeffs_ );
}

void RectifyHandle::publishToTopics() {
    ROS_INFO_NAMED(kPringName, "Init publishers");
    assert (initialized_);

    // Monitor whether anyone is subscribed to the output
    ros::SubscriberStatusCallback connect_cb = boost::bind(&RectifyHandle::connectCb, this);

    // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
    pub_rect_  = nh_.advertise<sensor_msgs::Image>(rec_publisher_topic_name_,  1, connect_cb, connect_cb);
    pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, false);
}

void RectifyHandle::connectCb() {
    ROS_INFO_NAMED(kPringName, "Init subscribers");
    assert (initialized_);

    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    std::cout << "SUBS num: " << pub_rect_.getNumSubscribers() << std::endl;
    if (pub_rect_.getNumSubscribers() == 0){
        if(sub_camera_){ // if subscriber is active
            sub_camera_.shutdown();
        }        
    }
    else if (!sub_camera_)
    {        
        sub_camera_ = nh_.subscribe(camera_subscriber_topic_name_, queue_size_, &RectifyHandle::imageCb, this);
    }
}

void RectifyHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){    
    // Convert to Opencs
    //cv_bridge::CvImagePtr cv_ptr; // use cv_bridge::toCvCopy with this 
    cv_bridge::CvImageConstPtr cv_ptr; 
    try
    {
        if (enc::isColor(msg->encoding))
          cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else
          cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImagePtr out_img(new cv_bridge::CvImage());    
    auto start_time = ros::Time::now();
    try{
        out_img->header   = cv_ptr->header;
        out_img->encoding = cv_ptr->encoding; 
        if(default_implementation_){
            ROS_INFO_NAMED(kPringName, "Call default function");
            professor::imageUndistort(cv_ptr->image, out_img->image, 
                            camera_matrix_, dist_coeffs_);
        }else{
            // CALL STUDENT FUNCTION    
            ROS_WARN_NAMED(kPringName, "Call student function");
            student::imageUndistort(cv_ptr->image, out_img->image, 
                            camera_matrix_, dist_coeffs_);
        }
    }catch(...){

    }
    std_msgs::Float32 dt_msg;    
    dt_msg.data = (ros::Time::now() - start_time).toSec();
    pub_dt_.publish(dt_msg);    
    pub_rect_.publish(out_img->toImageMsg());
}