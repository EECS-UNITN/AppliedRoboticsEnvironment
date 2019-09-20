#include "grab_frame_handle.hpp"
#include "student_interface.hpp"
#include "professor_interface.hpp"

#include "image_elab/ExtrinsicParams.h"
#include "image_elab/PlaneTransform.h"
#include "std_msgs/Float32.h"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "extrinsic_handle.hpp";

namespace image_proc {

    // Constructor
    GrabFrameHandle::GrabFrameHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_ = false;        
    }

    void GrabFrameHandle::onInit(ros::NodeHandle &nodeHandle){
        nh_ = nodeHandle;                

        loadParameters();         
        publishToTopics();        
        timer_ = nh_.createTimer(ros::Duration(1/freq_), &GrabFrameHandle::grabCb, this);        
        timer_.stop();        
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
    void GrabFrameHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");
              
        loadVariable<std::string>(nh_,"/config_folder",&config_folder_); 
              
        loadVariable<bool>(nh_,"/default_implementation/grab_frame", &default_implementation_);

        queue_size_ = 1;
        
        pub_img_topic_name_ = "/transform/ground_plane";        
        pub_dt_topic_name_  = "/process_time/grabFrame";
        freq_ = 10;
        initialized_ = true;
    }

  void GrabFrameHandle::connectCb() {
    ROS_DEBUG_NAMED(kPringName, "Init connectCb");
    assert (initialized_);    
    boost::lock_guard<boost::mutex> lock(connect_mutex_);    
    if (pub_img_.getNumSubscribers() == 0){         
        timer_.stop();        
    }
    else
    {   
        timer_.start();
    }
}


    void GrabFrameHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        ros::SubscriberStatusCallback connect_cb = boost::bind(&GrabFrameHandle::connectCb, this);
        pub_img_  = nh_.advertise<sensor_msgs::Image>(pub_img_topic_name_,  1, connect_cb, connect_cb);

        // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, false);
    }

    void GrabFrameHandle::grabCb(const ros::TimerEvent& ){
        cv_bridge::CvImagePtr out_img(new cv_bridge::CvImage());
        auto start_time = ros::Time::now();
        try{
            if(default_implementation_){
                ROS_DEBUG_NAMED(kPringName, "Call default function");
                professor::grabFrame(out_img->image, config_folder_);
            }else{
                // CALL STUDENT FUNCTION    
                ROS_WARN_NAMED(kPringName, "Call student function");
                student::grabFrame(out_img->image, config_folder_);
            }
        }catch(std::exception& ex){
          std::cerr << ex.what() << std::endl;
        }        

        std_msgs::Float32 dt_msg;
        dt_msg.data = (ros::Time::now() - start_time).toSec();
        pub_dt_.publish(dt_msg);
        
        // Publish the msg
        sensor_msgs::ImagePtr out_img_ros = out_img->toImageMsg();
        pub_img_.publish(out_img_ros);                     
    }
}
