#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <grab_frame_handle.hpp>

namespace image_elab
{

	class GrabFrameNodelet : public nodelet::Nodelet
	{
	public:
		GrabFrameNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getMTNodeHandle();
			handle_.onInit(nh);
	  	}

	  	image_proc::GrabFrameHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(image_elab::GrabFrameNodelet, nodelet::Nodelet)
}