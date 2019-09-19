#include "robot_detector_handle.hpp"
#include "student_interface.hpp"
#include "professor_interface.hpp"

#include "image_elab/ExtrinsicParams.h"
#include "image_elab/PlaneTransform.h"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>

#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Float32.h"

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "robot_detector_handle.hpp";

namespace image_proc {

    // Constructor
    RobotDetectorHandle::RobotDetectorHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_  = false;  
        has_transform_ = false;  
        
    }

    void RobotDetectorHandle::onInit(ros::NodeHandle &nodeHandle){
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
    void RobotDetectorHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");
      
              
        loadVariable<bool>(nh_,"/default_implementation/robot_detector", &default_implementation_);
        loadVariable<std::string>(nh_,"/config_folder",&config_folder_);
        
        queue_size_ = 1;

        sub_transf_topic_name_ = "/transform/robot_plane";
        sub_image_topic_name_  = "/image/unwarp_robot";
        
        pub_robot_topic_name_   = "/detection/robot";
        pub_gps_loc_topic_name_ = "/estimation/pose";
        pub_dt_topic_name_      = "/process_time/findRobot";

        frame_id_ = "map";
    }

    void RobotDetectorHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        pub_robot_ = nh_.advertise<geometry_msgs::PolygonStamped>(pub_robot_topic_name_, 1, false);
        pub_gps_loc_ = nh_.advertise<geometry_msgs::PoseStamped>(pub_gps_loc_topic_name_, 1, false);
        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, false);
    }

    void RobotDetectorHandle::subscribeToTopic() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
         
        sub_image_ = nh_.subscribe(sub_image_topic_name_, queue_size_, &RobotDetectorHandle::imageCb, this);

        sub_transf_ = nh_.subscribe(sub_transf_topic_name_, queue_size_, &RobotDetectorHandle::transformCb, this);
    }



    void RobotDetectorHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){   
        triangle_.clear();

        if (!has_transform_) return;

        // Convert to Opencs
        cv_bridge::CvImageConstPtr cv_ptr; 
        try
        {
            if (enc::isColor(msg->encoding))
              cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
            else
              cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);        
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        

        bool res = false;
        auto start_time = ros::Time::now();
        try{
            if(default_implementation_){
                ROS_DEBUG_NAMED(kPringName, "Call default function");

                // PROFESSOR FUNCTION IMPLEMENTATION
                res = professor::findRobot(cv_ptr->image, scale_, triangle_, x_, y_, theta_, config_folder_);

                 
            }else{
                // CALL STUDENT FUNCTION    
                ROS_DEBUG_NAMED(kPringName, "Call student function");
                
                // STUDENT FUNCTION IMPLEMENTATION
                res = student::findRobot(cv_ptr->image, scale_, triangle_, x_, y_, theta_, config_folder_);
            }

        }catch(std::exception& ex){
            std::cerr << ex.what() << std::endl;
        }
        std_msgs::Float32 dt_msg;
        dt_msg.data = (ros::Time::now() - start_time).toSec();
        pub_dt_.publish(dt_msg);
        
        if(res){

            geometry_msgs::PolygonStamped triangle_msg;
            geometry_msgs::PoseStamped robot_pose_msg;

            triangle_msg.header.stamp = msg->header.stamp;
            triangle_msg.header.frame_id = frame_id_;
            triangle_msg.polygon = createPolygon(triangle_);

    
    
            robot_pose_msg.header.stamp = msg->header.stamp;
            robot_pose_msg.header.frame_id = frame_id_;

            //set the position
            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

            robot_pose_msg.pose.position.x = x_;
            robot_pose_msg.pose.position.y = y_;
            robot_pose_msg.pose.position.z = 0.0;
            robot_pose_msg.pose.orientation = odom_quat; 
                       
            pub_robot_.publish(triangle_msg);
            pub_gps_loc_.publish(robot_pose_msg);
        }else{
            ROS_WARN_NAMED(kPringName, "findRobot returned false");
        }
    }

    void RobotDetectorHandle::transformCb(const image_elab::PlaneTransform& transf){
        scale_ = transf.scale;
        transform_ = cv::Mat(3, 3, CV_64F);
        assert(transf.matrix.size()==9);
        for (int i=0; i<transf.matrix.size(); ++i) {
            const int r = i/3;
            const int c = i%3;
            transform_.at<double>(r, c) = transf.matrix[i];
        }
        has_transform_ = true;        
    }

}

