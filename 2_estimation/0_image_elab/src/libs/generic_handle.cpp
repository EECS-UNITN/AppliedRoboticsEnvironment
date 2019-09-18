#include "generic_handle.hpp"
#include "student_interface.hpp"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
using namespace image_proc;

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "generic_handle.hpp";

// Constructor
GenericHandle::GenericHandle(){
    ROS_INFO_NAMED(kPringName, "Constructor");
    initialized_  = false;    
}

void GenericHandle::onInit(ros::NodeHandle &nodeHandle, std::string default_implementation_field, std::string camera_subscriber_topic_name){
    nh_ = nodeHandle;
    initialized_ = true;

    default_implementation_field_ = default_implementation_field; 
    //"/default_implementation/image_raw_clbk";
    camera_subscriber_topic_name_  = camera_subscriber_topic_name; 
    //"/camera/rgb/image_raw";  

    loadParameters();  

    if(default_implementation_){

        std::exit(0);
    }
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
void GenericHandle::loadParameters() {
    ROS_INFO_NAMED(kPringName, "Loading Params");
  
    loadVariable<std::string>(nh_,"/config_folder",&config_folder_);
    loadVariable<bool>(nh_,default_implementation_field_,&default_implementation_);  
}

void GenericHandle::connectCb() {
    ROS_INFO_NAMED(kPringName, "Init subscribers");
    assert (initialized_);

    sub_camera_ = nh_.subscribe(camera_subscriber_topic_name_, queue_size_, &GenericHandle::imageCb, this);    
}

void GenericHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){    
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
    
    try{     
        ROS_INFO_NAMED(kPringName, "calling student implementation");    
        student::genericImageListener(cv_ptr->image, camera_subscriber_topic_name_, config_folder_);
    }catch(...){

    }    
    
}