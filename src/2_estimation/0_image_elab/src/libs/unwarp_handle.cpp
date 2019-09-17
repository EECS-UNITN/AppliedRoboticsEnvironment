#include "unwarp_handle.hpp"
#include "student_interface.hpp"
#include "professor_interface.hpp"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "rectify_handle.hpp";


namespace image_proc {

    // Constructor
    UnwarpHandle::UnwarpHandle(){
        ROS_INFO_NAMED(kPringName, "Constructor");
        initialized_  = false;    
        has_ground_transf_ = has_robot_transf_ = false;
    }

    void UnwarpHandle::onInit(ros::NodeHandle &nodeHandle){
        nh_ = nodeHandle;
        initialized_ = true;

        loadParameters();    
        publishToTopics();
        subscribeToTopics();
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
    void UnwarpHandle::loadParameters() {
        ROS_INFO_NAMED(kPringName, "Loading Params");

        loadVariable<bool>(nh_,"/default_implementation/unwarp",&default_implementation_);


        sub_ground_topic_name_           = "/transform/ground_plane";
        sub_robot_topic_name_            = "/transform/robot_plane";
        sub_undistort_topic_name_        = "/image/rectify";
        robot_publisher_topic_name_      = "/image/unwarp_robot";
        ground_publisher_topic_name_     = "/image/unwarp_ground";
     
    }

    void UnwarpHandle::publishToTopics() {
        ROS_INFO_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        
        // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
        pub_unwarp_robot_  = nh_.advertise<sensor_msgs::Image>(robot_publisher_topic_name_,  1);

        pub_unwarp_ground_  = nh_.advertise<sensor_msgs::Image>(ground_publisher_topic_name_,  1);


    }

    void UnwarpHandle::subscribeToTopics() {
        ROS_INFO_NAMED(kPringName, "Init subscribers");
        assert (initialized_);

        sub_undistort_ = nh_.subscribe(sub_undistort_topic_name_, queue_size_, &UnwarpHandle::imageCb, this);
        sub_ground_ = nh_.subscribe(sub_ground_topic_name_, queue_size_, &UnwarpHandle::groundCb, this);
        sub_robot_ = nh_.subscribe(sub_robot_topic_name_, queue_size_, &UnwarpHandle::robotCb, this);
    }

    void UnwarpHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){    

        if (!has_robot_transf_ || !has_ground_transf_)
            return;


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

        cv_bridge::CvImagePtr out_img_ground(new cv_bridge::CvImage());
        cv_bridge::CvImagePtr out_img_robot(new cv_bridge::CvImage());
        try{
            out_img_ground->header   = cv_ptr->header;
            out_img_ground->encoding = cv_ptr->encoding; 
            out_img_robot->header    = cv_ptr->header;
            out_img_robot->encoding  = cv_ptr->encoding; 
            if(default_implementation_){
                ROS_INFO_NAMED(kPringName, "Call default function");
                professor::unwarp(cv_ptr->image, out_img_ground->image, ground_transf_, scale_ground_);
                professor::unwarp(cv_ptr->image, out_img_robot->image, robot_transf_, scale_robot_);
            }else{
                // CALL STUDENT FUNCTION    
                ROS_WARN_NAMED(kPringName, "Call student function");
                student::unwarp(cv_ptr->image, out_img_ground->image, ground_transf_, scale_ground_);
                student::unwarp(cv_ptr->image, out_img_robot->image, robot_transf_, scale_robot_);
            }
        }catch(...){

        }
        // IMPORTANT PUBLISH SHARED POINTER!!!
        sensor_msgs::ImagePtr u_ground = out_img_ground->toImageMsg();
        pub_unwarp_ground_.publish(u_ground);
        sensor_msgs::ImagePtr u_robot = out_img_robot->toImageMsg();
        pub_unwarp_robot_.publish(u_robot);
    }

    void UnwarpHandle::groundCb(const image_elab::PlaneTransform& transf){
        scale_ground_ = transf.scale;
        ground_transf_ = cv::Mat(3, 3, CV_64F);
        assert(transf.matrix.size()==9);
        for (int i=0; i<transf.matrix.size(); ++i) {
            const int r = i/3;
            const int c = i%3;
            ground_transf_.at<double>(r, c) = transf.matrix[i];
        }
        has_ground_transf_ = true;
    }

    void UnwarpHandle::robotCb(const image_elab::PlaneTransform& transf){
        scale_robot_ = transf.scale;
        robot_transf_ = cv::Mat(3, 3, CV_64F);
        assert(transf.matrix.size()==9);
        for (int i=0; i<transf.matrix.size(); ++i) {
            const int r = i/3;
            const int c = i%3;
            robot_transf_.at<double>(r, c) = transf.matrix[i];
        }
        has_robot_transf_ = true;
    }
    
}