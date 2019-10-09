#include "unwarp_handle.hpp"
#include "student_image_elab_interface.hpp"
#include "professor_image_elab_interface.hpp"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32.h"
#include <sstream>

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "unwarp_handle.hpp";


namespace image_proc {

    // Constructor
    UnwarpHandle::UnwarpHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_  = false;    
        has_ground_transf_ = has_robot_transf_ = false;
    }

    void UnwarpHandle::onInit(ros::NodeHandle &nodeHandle){
        nh_ = nodeHandle;
        initialized_ = true;

        loadParameters();    
        publishToTopics();
        subscribeToTopics();

        // Be sure that the variable are initialized properly
        connectGroundCb();
        connectRobotCb();
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
        ROS_DEBUG_NAMED(kPringName, "Loading Params");

        loadVariable<bool>(nh_,"/default_implementation/unwarp",&default_implementation_);
        loadVariable<std::string>(nh_,"/config_folder",&config_folder_);

        sub_ground_topic_name_           = "/transform/ground_plane";
        sub_robot_topic_name_            = "/transform/robot_plane";
        sub_undistort_topic_name_        = "/image/rectify";
        robot_publisher_topic_name_      = "/image/unwarp_robot";
        ground_publisher_topic_name_     = "/image/unwarp_ground";
        pub_dt_topic_name_               = "/process_time/unwarp";     
    }

    void UnwarpHandle::connectGroundCb(){
        n_subscribers_ground_ = pub_unwarp_ground_.getNumSubscribers();
    }

    void UnwarpHandle::connectRobotCb(){
        n_subscribers_robot_ = pub_unwarp_robot_.getNumSubscribers();
    }

    void UnwarpHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        // Monitor whether anyone is subscribed to the output
        ros::SubscriberStatusCallback connect_ground_cb = boost::bind(&UnwarpHandle::connectGroundCb, this);
        ros::SubscriberStatusCallback connect_robot_cb = boost::bind(&UnwarpHandle::connectRobotCb, this);

        // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
        pub_unwarp_robot_  = nh_.advertise<sensor_msgs::Image>(robot_publisher_topic_name_,  1, connect_robot_cb, connect_robot_cb);

        pub_unwarp_ground_  = nh_.advertise<sensor_msgs::Image>(ground_publisher_topic_name_,  1, connect_ground_cb, connect_ground_cb);

        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, false);

    }

    void UnwarpHandle::subscribeToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
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
        auto start_time = ros::Time::now();
        try{
            out_img_ground->header   = cv_ptr->header;
            out_img_ground->encoding = cv_ptr->encoding; 
            out_img_robot->header    = cv_ptr->header;
            out_img_robot->encoding  = cv_ptr->encoding; 
            if(default_implementation_){
                ROS_DEBUG_NAMED(kPringName, "Call default function");
                if(n_subscribers_ground_ > 0){
                    professor::unwarp(cv_ptr->image, out_img_ground->image, ground_transf_, config_folder_);
                }
                if(n_subscribers_robot_ > 0){
                    professor::unwarp(cv_ptr->image, out_img_robot->image, robot_transf_, config_folder_);
                }
            }else{
                // CALL STUDENT FUNCTION    
                ROS_DEBUG_NAMED(kPringName, "Call student function");
                if(n_subscribers_ground_ > 0){
                    student::unwarp(cv_ptr->image, out_img_ground->image, ground_transf_, config_folder_);
                }
                if(n_subscribers_robot_ > 0){
                    student::unwarp(cv_ptr->image, out_img_robot->image, robot_transf_, config_folder_);
                }
            }
        }catch(std::exception& ex){
            std::cerr << ex.what() << std::endl;
        }
        std_msgs::Float32 dt_msg;
        dt_msg.data = (ros::Time::now() - start_time).toSec();
        pub_dt_.publish(dt_msg); 
               
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
