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

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "robot_detector_handle.hpp";

namespace image_proc {

    // Constructor
    RobotDetectorHandle::RobotDetectorHandle(){
        ROS_INFO_NAMED(kPringName, "Constructor");
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
        ROS_INFO_NAMED(kPringName, "Loading Params");
      
              
        loadVariable<bool>(nh_,"/default_implementation/robot_detector", &default_implementation_);


        queue_size_ = 1;

        sub_transf_topic_name_ = "/transform/robot_plane";
        sub_image_topic_name_  = "/image/unwarp_robot";
        
        pub_robot_topic_name_   = "/detection/robot";
        pub_gps_loc_topic_name_ = "/estimation/pose";
        
    }

    void RobotDetectorHandle::publishToTopics() {
        ROS_INFO_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        pub_robot_ = nh_.advertise<geometry_msgs::PolygonStamped>(pub_robot_topic_name_, 1, false);
        pub_gps_loc_ = nh_.advertise<geometry_msgs::PoseStamped>(pub_gps_loc_topic_name_, 1, false);
    }

    void RobotDetectorHandle::subscribeToTopic() {
        ROS_INFO_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
         
        sub_image_ = nh_.subscribe(sub_image_topic_name_, queue_size_, &RobotDetectorHandle::imageCb, this);

        sub_transf_ = nh_.subscribe(sub_transf_topic_name_, queue_size_, &RobotDetectorHandle::transformCb, this);
    }



    geometry_msgs::Polygon createPolygon(const Polygon & poly_2D){

        geometry_msgs::Polygon poly_3D;
        for (const auto & pt: poly_2D){ 
            geometry_msgs::Point32 pt_3D;
            pt_3D.x = pt.x;
            pt_3D.y = pt.y;
            pt_3D.z = 0.;
            poly_3D.points.push_back(pt_3D);
        }
        return poly_3D;
    }
                



    void RobotDetectorHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){   
        triangle_.clear();

        if (!has_transform_) return;

        auto delta_time = (ros::Time::now() - msg->header.stamp).toSec();
        ROS_WARN_STREAM("Elapsed time " << delta_time);
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
        try{
            if(default_implementation_){
                ROS_INFO_NAMED(kPringName, "Call default function");

                // PROFESSOR FUNCTION IMPLEMENTATION
                res = professor::findRobot(cv_ptr->image, scale_, triangle_, x_, y_, theta_);

                 
            }else{
                // CALL STUDENT FUNCTION    
                ROS_WARN_NAMED(kPringName, "Call student function");
                
                // STUDENT FUNCTION IMPLEMENTATION
                res = student::findRobot(cv_ptr->image, scale_, triangle_, x_, y_, theta_);
            }

        }catch(...){

        }

        if(res){

            std::string frame_id = "odom";
            
            geometry_msgs::PolygonStamped triangle_msg;
            geometry_msgs::PoseStamped robot_pose_msg;

            triangle_msg.header.stamp = msg->header.stamp;
            triangle_msg.header.frame_id = frame_id;
            triangle_msg.polygon = createPolygon(triangle_);

    
    
            robot_pose_msg.header.stamp = msg->header.stamp;
            robot_pose_msg.header.frame_id = frame_id;

            //set the position
            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

            robot_pose_msg.pose.position.x = x_;
            robot_pose_msg.pose.position.y = y_;
            robot_pose_msg.pose.position.z = 0.0;
            robot_pose_msg.pose.orientation = odom_quat; 
                       
            pub_robot_.publish(triangle_msg);
            pub_gps_loc_.publish(robot_pose_msg);
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

