#include "extrinsic_handle.hpp"
#include "student_image_elab_interface.hpp"
#include "professor_image_elab_interface.hpp"

#include "image_elab/ExtrinsicParams.h"
#include "image_elab/PlaneTransform.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include <sstream>

namespace enc = sensor_msgs::image_encodings;
const std::string kPringName = "extrinsic_handle.hpp";

namespace image_proc {

    // Constructor
    ExtrinsicHandle::ExtrinsicHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_  = false;    
    }

    void ExtrinsicHandle::onInit(ros::NodeHandle &nodeHandle){
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
    void ExtrinsicHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");
      
        double fx, fy, cx, cy;
        loadVariable<std::string>(nh_,"/config_folder",&config_folder_); 
        loadVariable<double>(nh_, "/camera_calibration/fx", &fx);
        loadVariable<double>(nh_, "/camera_calibration/fy", &fy);
        loadVariable<double>(nh_, "/camera_calibration/cx", &cx);
        loadVariable<double>(nh_, "/camera_calibration/cy", &cy);
      
        loadVariable<bool>(nh_,"/default_implementation/extrinsic_calib", &default_implementation_);
        queue_size_ = 1;
        extr_publisher_topic_name_   = "/transform/camera_pose";
        rect_subscriber_topic_name_  = "/image/rectify";//"/camera/rgb/image_raw"; 
        //camera_subscriber_topic_name_  = "/camera/rgb/image_raw";
        pub_ground_topic_name_ = "/transform/ground_plane";
        pub_robot_topic_name_  = "/transform/robot_plane";
        pub_dt_topic_name_     = "/process_time/findPlaneTransform";
        pub_camera_pose_rviz_topic_name_ = "/estimation/camera_pose";

        double arena_w, arena_h, robot_height;
        int img_width, img_height;

        loadVariable<int>(nh_,"/camera_calibration/image_height",&img_height);
        loadVariable<int>(nh_,"/camera_calibration/image_width",&img_width);
        loadVariable<double>(nh_,"/arena/w",&arena_w);
        loadVariable<double>(nh_,"/arena/h",&arena_h);
        loadVariable<double>(nh_,"/arena/robot_height",&robot_height);
        scale_ = std::min(img_height/arena_h, img_width/arena_w);


        calib_done_ = false;
        float w = arena_w*scale_, h = arena_h*scale_, z = robot_height;
        object_points_ground_ = {{0.f,0.f,0.f},{w,0.f,0.f},{w,h,0.f},{0.f,h,0.f}};
        object_points_robot_ = {{0.f,0.f,z},{w,0.f,z},{w,h,z},{0.f,h,z}};

        camera_matrix_ = (cv::Mat1d(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

        ROS_INFO_STREAM_NAMED(kPringName, "cametra_matrix = \n " << camera_matrix_);
    }

    void ExtrinsicHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);


        // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
        pub_extr_  = nh_.advertise<image_elab::ExtrinsicParams>(extr_publisher_topic_name_, 1, true);
        pub_ground_transf_ = nh_.advertise<image_elab::PlaneTransform>(pub_ground_topic_name_, 1, true);
        pub_robot_transf_ = nh_.advertise<image_elab::PlaneTransform>(pub_robot_topic_name_, 1, true);
        pub_camera_pose_rviz_ = nh_.advertise<geometry_msgs::PoseStamped>(pub_camera_pose_rviz_topic_name_, 1, true);
        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, false);
    }

    void ExtrinsicHandle::subscribeToTopic() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
         
        sub_rect_ = nh_.subscribe(rect_subscriber_topic_name_, queue_size_, &ExtrinsicHandle::imageCb, this);

    }


    void ExtrinsicHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){    
        
        if (calib_done_) return;    

        // Convert to Opencs
        //cv_bridge::CvImagePtr cv_ptr; // use cv_bridge::toCvCopy with this 
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
        
        cv_bridge::CvImagePtr out_img(new cv_bridge::CvImage());
        image_elab::ExtrinsicParamsPtr extr_msg(new image_elab::ExtrinsicParams());
        image_elab::PlaneTransformPtr ground_msg(new image_elab::PlaneTransform());
        image_elab::PlaneTransformPtr robot_msg(new image_elab::PlaneTransform());

        bool res = false;
        auto start_time = ros::Time::now();        
        try{
            if(default_implementation_){
                ROS_DEBUG_NAMED(kPringName, "Call default function");
                res = professor::extrinsicCalib(cv_ptr->image, object_points_ground_, camera_matrix_, rvec_, tvec_, config_folder_);
                
                if (res){
                    professor::findPlaneTransform(camera_matrix_, rvec_, tvec_, object_points_ground_, ground_plane_, config_folder_);
                    
                    professor::findPlaneTransform(camera_matrix_, rvec_, tvec_, object_points_robot_, robot_plane_, config_folder_);            
                }
                
            }else{
                // CALL STUDENT FUNCTION    
                ROS_WARN_NAMED(kPringName, "Call student function");
                res = student::extrinsicCalib(cv_ptr->image, object_points_ground_, camera_matrix_, rvec_, tvec_, config_folder_);            
                if (res){
                    student::findPlaneTransform(camera_matrix_, rvec_, tvec_, object_points_ground_, ground_plane_, config_folder_);
                    student::findPlaneTransform(camera_matrix_, rvec_, tvec_, object_points_robot_, robot_plane_, config_folder_);
                }
            }
        }catch(std::exception& ex){
          std::cerr << ex.what() << std::endl;
        }        

        std_msgs::Float32 dt_msg;
        dt_msg.data = (ros::Time::now() - start_time).toSec();
        pub_dt_.publish(dt_msg);
        if(res){
          for (int i=0; i<robot_plane_.rows; ++i) {
              for (int j=0; j<robot_plane_.cols; ++j) {
              robot_msg->matrix.push_back(robot_plane_.at<double>(i,j));
              }      
          }        
          robot_msg->scale = scale_;
          for (int i=0; i<ground_plane_.rows; ++i) {
              for (int j=0; j<ground_plane_.cols; ++j) {
              ground_msg->matrix.push_back(ground_plane_.at<double>(i,j));
              }      
          }
          ground_msg->scale = scale_;

          for (int i=0; i<rvec_.rows; ++i) {
              for (int j=0; j<rvec_.cols; ++j) {
              extr_msg->rvec.push_back(rvec_.at<double>(i,j));
              }      
          }

          for (int i=0; i<tvec_.rows; ++i) {
              for (int j=0; j<tvec_.cols; ++j) {
              extr_msg->tvec.push_back(tvec_.at<double>(i,j));
              }      
          }
          sub_rect_.shutdown();

          // Compute camera pose in ground coordinate
          cv::Mat Rt;
          cv::Rodrigues(rvec_, Rt);
          cv::Mat R = Rt.t();
          cv::Mat camera_pt = -R * tvec_;
          std::cout << camera_pt << std::endl;
          tf2::Matrix3x3 R_conversion(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
          tf2::Quaternion camera_quat;
          geometry_msgs::PoseStamped camera_pose_msg;
          camera_pose_msg.header.stamp = ros::Time::now();
          camera_pose_msg.header.frame_id = "map";
          camera_pose_msg.pose.position.x = camera_pt.at<double>(0)/1000.;
          camera_pose_msg.pose.position.y = camera_pt.at<double>(1)/1000.;
          camera_pose_msg.pose.position.z = camera_pt.at<double>(2)/1000.;
          
          R_conversion.getRotation(camera_quat);
          camera_pose_msg.pose.orientation.w = camera_quat.w();
          camera_pose_msg.pose.orientation.x = camera_quat.x();
          camera_pose_msg.pose.orientation.y = camera_quat.y();
          camera_pose_msg.pose.orientation.z = camera_quat.z()  ;
          
          // Publish the msg
          pub_extr_.publish(extr_msg);  
          pub_ground_transf_.publish(ground_msg);
          pub_robot_transf_.publish(robot_msg);
          pub_camera_pose_rviz_.publish(camera_pose_msg);
          calib_done_ = true;
        }else{
          ROS_WARN_NAMED(kPringName, "findPlaneTransform returned false");
        }
    }
}
