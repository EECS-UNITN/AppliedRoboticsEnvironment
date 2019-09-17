#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <robot_detector_handle.hpp>

namespace image_elab
{

	class RobotDetectorNodelet : public nodelet::Nodelet
	{
	public:
		RobotDetectorNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getMTNodeHandle();
			handle_.onInit(nh);
	  	}

	  	image_proc::RobotDetectorHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(image_elab::RobotDetectorNodelet, nodelet::Nodelet)
}