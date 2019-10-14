#include "planning_handle.hpp"
#include "student_planning_interface.hpp"
#include "professor_planning_interface.hpp"

#include "std_msgs/Float32.h" // used to publish dt
#include "nav_msgs/Path.h"
#include "planning/WaypointList.h"
#include "tf/transform_broadcaster.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <assert.h>
#include <sstream>


const std::string kPringName = "planning_handle.hpp";

namespace planning {

    // Constructor
    PlanningHandle::PlanningHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_   = false;  
        has_victims_   = false;
        has_obstacles_ = false;
        has_gate_      = false;
        has_robot_     = false;
    }

    void PlanningHandle::onInit(ros::NodeHandle &nodeHandle){
        nh_ = nodeHandle;
        initialized_ = true;

        loadParameters();    
        publishToTopics();
        subscribeToTopic();
        initServices();
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
    void PlanningHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");
      
        
        float arena_w, arena_h;

        loadVariable<bool>(nh_,"/default_implementation/planning", &default_implementation_);
        loadVariable<std::string>(nh_,"/config_folder",&config_folder_);
        loadVariable<float>(nh_,"/arena/w",&arena_w);
        loadVariable<float>(nh_,"/arena/h",&arena_h);

        queue_size_ = 1;

        sub_obstacles_topic_name_ = "/detection/obstacles";
        sub_victims_topic_name_   = "/detection/victims";
        sub_gate_topic_name_      = "/detection/gate";
        sub_robot_topic_name_     = "/estimation/pose";

        pub_plan_topic_name_ = "/planning/plan";
        pub_plan_rviz_topic_name_ = "/planning/plan_rviz";
        pub_dt_topic_name_   = "/process_time/planning";

        srv_plan_topic_name_ = "/planning/compute_plan";

        borders_ = {{0.f,0.f},{arena_w,0.f},{arena_w,arena_h},{0.f,arena_h}};
    }

    void PlanningHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);

        pub_plan_ = nh_.advertise<planning::WaypointList>(pub_plan_topic_name_, 1, true);
        //pub_plan_rviz_ = nh_.advertise<nav_msgs::Path>(pub_plan_rviz_topic_name_, 1, true);
        pub_plan_rviz_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_plan_rviz_topic_name_, 1, true);        
        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, true);
    }

    void PlanningHandle::subscribeToTopic() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
         
        sub_victims_ = nh_.subscribe(sub_victims_topic_name_, queue_size_, &PlanningHandle::victimsCb, this);
        sub_obstacles_ = nh_.subscribe(sub_obstacles_topic_name_, queue_size_, &PlanningHandle::obstaclesCb, this);
        sub_gate_ = nh_.subscribe(sub_gate_topic_name_, queue_size_, &PlanningHandle::gateCb, this);
        sub_robot_ = nh_.subscribe(sub_robot_topic_name_, queue_size_, &PlanningHandle::robotCb, this);
    }

    void PlanningHandle::initServices(){
        ROS_DEBUG_NAMED(kPringName, "Init services");
        assert (initialized_);

        srv_plan_ = nh_.advertiseService(srv_plan_topic_name_, &PlanningHandle::computePlanSrv, this);
    }


    Polygon createPolygon(const geometry_msgs::Polygon & poly){
        Polygon res;
        for (const auto & pt: poly.points){ 
            res.emplace_back(pt.x, pt.y);
        }
        return res;
    }
        

    void PlanningHandle::robotCb(const geometry_msgs::PoseStampedPtr robot_pose){
        x_ = robot_pose->pose.position.x;
        y_ = robot_pose->pose.position.y;
        tf::Quaternion q(robot_pose->pose.orientation.x, 
            robot_pose->pose.orientation.y, 
            robot_pose->pose.orientation.z, 
            robot_pose->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_ = yaw;
        has_robot_ = true;
    }

    void PlanningHandle::victimsCb(const jsk_recognition_msgs::PolygonArrayPtr victims){
        victim_list_.clear();
        for (int i=0; i<victims->polygons.size(); ++i){
            const auto & poly = victims->polygons[i].polygon;
            victim_list_.emplace_back(victims->labels[i], createPolygon(poly));
        }
        has_victims_ = true;
    }

    void PlanningHandle::obstaclesCb(const jsk_recognition_msgs::PolygonArrayPtr obstacles){
        obstacle_list_.clear();
        for (const auto & poly: obstacles->polygons){
            obstacle_list_.emplace_back(createPolygon(poly.polygon));
        }
        header_ = obstacles->header;
        has_obstacles_ = true;
    }

    void PlanningHandle::gateCb(const jsk_recognition_msgs::PolygonArrayPtr gate){
        gate_.clear();
        if (gate->polygons.size() != 1) {
            throw std::runtime_error("Gate size != 1: " + std::to_string(gate->polygons.size()));
        }
        gate_ = createPolygon(gate->polygons[0].polygon);
        has_gate_ = true;
    }



    WaypointList createWPList(const Path& path, const std_msgs::Header& header){
        WaypointList res;
        res.header = header;
        for (const auto& pt: path.points) {
            Waypoint wp;
            wp.s = pt.s;
            wp.x = pt.x;
            wp.y = pt.y;
            wp.theta = pt.theta;
            wp.kappa = pt.kappa;
            res.wp.push_back(wp);
        }
        return res;
    }

    visualization_msgs::Marker markerHeader(int id, int type, std::string ns, double rgba[4], double dim[3]){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";

        switch(type){
            case 0:
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.scale.x = dim[0];
                break;
            
            case 1:
                marker.type = visualization_msgs::Marker::SPHERE_LIST;
                marker.scale.x = dim[0];
                marker.scale.y = dim[1];
                marker.scale.z = dim[2];
                break;

            case 2:
                marker.type = visualization_msgs::Marker::CUBE_LIST;    
                marker.scale.x = dim[0];
                marker.scale.y = dim[1];
                marker.scale.z = dim[2];
                break;

            case 3:
                marker.type = visualization_msgs::Marker::LINE_LIST;    
                marker.scale.x = dim[0];
                break;
    }

    marker.header.stamp = ros::Time::now();        
    marker.action = visualization_msgs::Marker::ADD;
    //visual_paths.markers[i].lifetime = ros::Duration(0.9);    
    marker.ns = ns;
    marker.color.r = rgba[0];
    marker.color.g = rgba[1];
    marker.color.b = rgba[2];
    marker.color.a = rgba[3];        
    marker.id = id;

    return marker;
} 


    void getInterpolatedWp(const WaypointList& path,
            visualization_msgs::Marker& markerCurve, 
            visualization_msgs::Marker& markerTangent,
            visualization_msgs::Marker& markerCurvature
             ){

        const double curvature_scale = 10;
        const double tangent_discr = 0.2;
        const double tangent_l = 0.1;

        double rgba[4] = {4/255., 150/255., 216/255., 0.5};
        double dim[3] = {0.005, 0.02, 0.02};
        markerCurvature = markerHeader(998, 3,"interp_wp_curvature", rgba, dim);
        rgba[0] = 0; rgba[1] = 0; rgba[2] = 1; rgba[3] = 0.75;
        markerCurve   = markerHeader(999, 0,"interp_wp", rgba, dim);    
        rgba[0] = 0; rgba[1] = 0; rgba[2] = 0; rgba[3] = 1;
        dim[0] = 0.005;
        markerTangent = markerHeader(997, 3,"interp_wp_tangent", rgba, dim);

        bool tangent_init = false;
        double s_last_tangent;
        for (const auto& wp: path.wp){
            const double x = wp.x;
            const double y = wp.y;
            geometry_msgs::Point p1, pk;
            p1.x = x;
            p1.y = y;
            p1.z = 0.002;
            markerCurve.points.push_back(p1);
            
            if(std::abs(wp.kappa) < 0.1){
                pk.x = x;
                pk.y = y;
            }else{
                pk.x = x + std::cos(wp.theta + M_PI/2.) / wp.kappa;
                pk.y = y + std::sin(wp.theta + M_PI/2.) / wp.kappa;            
            }
            pk.z = 0.002;

            markerCurvature.points.push_back(p1);
            markerCurvature.points.push_back(pk);
            if(!tangent_init || wp.s - s_last_tangent >  tangent_discr){
                tangent_init = true;
                s_last_tangent = wp.s;
                geometry_msgs::Point p2;
                p2.x = x + std::cos(wp.theta)*tangent_l;
                p2.y = y + std::sin(wp.theta)*tangent_l;
                p2.z = 0.002;              
                markerTangent.points.push_back(p1);
                markerTangent.points.push_back(p2);
            }
        }
    }


    visualization_msgs::MarkerArray createRVIZPath(const WaypointList& path){        
        visualization_msgs::MarkerArray arr;
        arr.markers.resize(3);
        getInterpolatedWp(path, arr.markers[0], arr.markers[1], arr.markers[2]);
        return arr;
    }


    // nav_msgs::Path createRVIZPath(const WaypointList& path){
    //     nav_msgs::Path res;
    //     res.header = path.header;
    //     for (const auto& pt: path.wp) {
    //         geometry_msgs::PoseStamped wp;
    //         wp.header = path.header;
    //         wp.pose.position.x = pt.x;
    //         wp.pose.position.y = pt.y;
    //         wp.pose.orientation = tf::createQuaternionMsgFromYaw(pt.theta);
    //         res.poses.push_back(wp);
    //     }
    //     return res;   
    // }

    bool PlanningHandle::computePlanSrv(ComputePlan::Request& req, ComputePlan::Response& res){
        if (!has_gate_ || !has_obstacles_ || !has_victims_ || !has_robot_) {
            res.status = -1; 
        }
        else {
            bool ok = false;
            try{
                if(default_implementation_){
                    ROS_DEBUG_NAMED(kPringName, "Call default function");
                    ok = professor::planPath(borders_, obstacle_list_, victim_list_, gate_, x_, y_, theta_, path_);
                }else{
                    // CALL STUDENT FUNCTION    
                    ROS_DEBUG_NAMED(kPringName, "Call student function");
                    ok = student::planPath(borders_, obstacle_list_, victim_list_, gate_, x_, y_, theta_, path_);
                }
            }catch(std::exception& ex){
                std::cerr << ex.what() << std::endl;
            }

            if (ok) {
                res.status = 0;
                res.wp = createWPList(path_, header_);
                pub_plan_.publish(res.wp);
                pub_plan_rviz_.publish(createRVIZPath(res.wp));
            }
            else {
                res.status = -2;
            }

        }
        return true;
    }
}

