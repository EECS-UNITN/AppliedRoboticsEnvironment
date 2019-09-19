#include "obstacle_detector_handle.hpp"
#include "student_interface.hpp"
#include "professor_interface.hpp"

#include "image_elab/ExtrinsicParams.h"
#include "image_elab/PlaneTransform.h"

#include <assert.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>

#include "jsk_recognition_msgs/PolygonArray.h"
#include "std_msgs/Float32.h"

namespace enc = sensor_msgs::image_encodings;
static const std::string kPringName = "obstacle_detector_handle.hpp";

namespace image_proc {

    // Constructor
    ObstacleDetectorHandle::ObstacleDetectorHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_  = false;  
        has_transform_ = false;  
    }

    void ObstacleDetectorHandle::onInit(ros::NodeHandle &nodeHandle){
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
    void ObstacleDetectorHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");
      
        loadVariable<std::string>(nh_,"/config_folder",&config_folder_);
        loadVariable<bool>(nh_,"/default_implementation/obstacle_detector", &default_implementation_);

        loadVariable<double>(nh_,"/arena/w",&arena_w_);
        loadVariable<double>(nh_,"/arena/h",&arena_h_);

        queue_size_ = 1;

        sub_transf_topic_name_ = "/transform/ground_plane";
        sub_image_topic_name_  = "/image/unwarp_ground";
        
        pub_obstacles_topic_name_ = "/detection/obstacles";
        pub_victims_topic_name_   = "/detection/victims";
        pub_gate_topic_name_      = "/detection/gate";
        pub_perimeter_topic_name_ = "/detection/perimeter";
        pub_dt_topic_name_        = "/process_time/processMap";

        frame_id_ = "map";
    }

    void ObstacleDetectorHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        pub_perimeter_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>(pub_perimeter_topic_name_, 1, true);
        pub_obstacles_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>(pub_obstacles_topic_name_, 1, true);
        pub_victims_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>(pub_victims_topic_name_, 1, true);
        pub_gate_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>(pub_gate_topic_name_, 1, true);
        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, false);
    }

    void ObstacleDetectorHandle::subscribeToTopic() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
         
        sub_image_ = nh_.subscribe(sub_image_topic_name_, queue_size_, &ObstacleDetectorHandle::imageCb, this);

        sub_transf_ = nh_.subscribe(sub_transf_topic_name_, queue_size_, &ObstacleDetectorHandle::transformCb, this);
    }


    void ObstacleDetectorHandle::imageCb(const sensor_msgs::ImageConstPtr& msg){    
        
        obstacle_list_.clear();
        victim_list_.clear();
        gate_.clear();

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
                res = professor::processMap(cv_ptr->image, scale_, obstacle_list_, victim_list_, gate_, config_folder_);

                 
            }else{
                // CALL STUDENT FUNCTION    
                ROS_DEBUG_NAMED(kPringName, "Call student function");
                
                // STUDENT FUNCTION IMPLEMENTATION
                res = student::processMap(cv_ptr->image, scale_, obstacle_list_, victim_list_, gate_, config_folder_);
            }

        }catch(std::exception& ex){
          std::cerr << ex.what() << std::endl;
        }
        std_msgs::Float32 dt_msg;
        dt_msg.data = (ros::Time::now() - start_time).toSec();
        pub_dt_.publish(dt_msg);
        
        if(res){
            
            jsk_recognition_msgs::PolygonArray obstacles_array;

            static int cnt = 0;

            obstacles_array.header.stamp = msg->header.stamp;
            obstacles_array.header.frame_id = frame_id_;
            obstacles_array.header.seq = cnt++;
            for (int i=0; i<obstacle_list_.size(); ++i) {
                geometry_msgs::PolygonStamped poly;
                poly.header = obstacles_array.header;
                poly.header.seq = cnt++;
                poly.polygon = createPolygon(obstacle_list_[i]);
                obstacles_array.polygons.push_back(poly);
                obstacles_array.labels.emplace_back(i);
                obstacles_array.likelihood.emplace_back(1.0);
            }


            jsk_recognition_msgs::PolygonArray victims_array;

            victims_array.header.stamp = msg->header.stamp;
            victims_array.header.frame_id = frame_id_;
            victims_array.header.seq = cnt++;
            for (int i=0; i<victim_list_.size(); ++i) {
                geometry_msgs::PolygonStamped poly;
                poly.header = victims_array.header;
                poly.header.seq = cnt++;
                poly.polygon = createPolygon(victim_list_[i].second);
                victims_array.polygons.push_back(poly);
                victims_array.labels.emplace_back(victim_list_[i].first);
                victims_array.likelihood.emplace_back(1.0);
            }

            jsk_recognition_msgs::PolygonArray gate_array;
            gate_array.header.stamp = msg->header.stamp;
            gate_array.header.frame_id = frame_id_;
            gate_array.header.seq = cnt++;
            geometry_msgs::PolygonStamped poly;
            poly.header = gate_array.header;
            poly.header.seq = cnt++;
            poly.polygon = createPolygon(gate_);
            gate_array.polygons.push_back(poly);
            gate_array.labels.emplace_back(0);
            gate_array.likelihood.emplace_back(1.0);
                 
            geometry_msgs::PolygonStamped poly_p;            
            geometry_msgs::Point32 pt;
            pt.x = 0; pt.y = 0; pt.z = 0;
            poly_p.polygon.points.push_back(pt);
            pt.x = arena_w_; pt.y = 0; pt.z = 0;
            poly_p.polygon.points.push_back(pt);
            pt.x = arena_w_; pt.y = arena_h_; pt.z = 0;
            poly_p.polygon.points.push_back(pt);
            pt.x = 0; pt.y = arena_h_; pt.z = 0;
            poly_p.polygon.points.push_back(pt);
            
            jsk_recognition_msgs::PolygonArray perimeter_array;
            perimeter_array.header.stamp = msg->header.stamp;
            perimeter_array.header.frame_id = frame_id_;
            perimeter_array.header.seq = cnt++;  
            poly_p.header = perimeter_array.header;
            poly_p.header.seq = cnt++;    
            perimeter_array.polygons.push_back(poly_p);
            perimeter_array.labels.emplace_back(0);
            perimeter_array.likelihood.emplace_back(1.0);

            
            
            pub_perimeter_.publish(perimeter_array);
            pub_obstacles_.publish(obstacles_array);
            pub_victims_.publish(victims_array);
            pub_gate_.publish(gate_array);

            // Distable the subscribers  
            sub_image_.shutdown();
            sub_transf_.shutdown();
        }else{
            ROS_WARN_NAMED(kPringName, "processMap returned false");
        }
    }

    void ObstacleDetectorHandle::transformCb(const image_elab::PlaneTransform& transf){
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

