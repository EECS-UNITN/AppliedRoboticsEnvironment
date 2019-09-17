#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <obstacle_detector_handle.hpp>

namespace image_elab
{

	class ObstacleDetectorNodelet : public nodelet::Nodelet
	{
	public:
		ObstacleDetectorNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getMTNodeHandle();
			handle_.onInit(nh);
	  	}

	  	image_proc::ObstacleDetectorHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(image_elab::ObstacleDetectorNodelet, nodelet::Nodelet)
}