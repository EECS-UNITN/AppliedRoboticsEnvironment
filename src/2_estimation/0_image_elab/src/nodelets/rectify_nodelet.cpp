#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rectify_handle.hpp>

namespace image_elab
{

	class RectifyNodelet : public nodelet::Nodelet
	{
	public:
		RectifyNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getMTNodeHandle();
			handle_.onInit(nh);
	  	}

	  	image_proc::RectifyHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(image_elab::RectifyNodelet, nodelet::Nodelet)
}