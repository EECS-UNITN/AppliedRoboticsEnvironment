#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "student_image_elab_interface.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/image/raw", 1);  

  std::string config_folder;
  if (!nh.getParam("/config_folder", config_folder)) 
  {          
      std::stringstream ss;
      
      ROS_ERROR_STREAM("Did not load " << "/config_folder");
      throw std::logic_error("Did not load /config_folder");
  }

  std::string frame_id = "camera";
  ros::Rate loop_rate(30);
  std_msgs::Header header;
  header.frame_id = frame_id;
  while (nh.ok()) {
    header.stamp = ros::Time::now();

    cv::Mat image;
    student::loadImage(image, config_folder);

    sensor_msgs::ImagePtr msg = 
              cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}