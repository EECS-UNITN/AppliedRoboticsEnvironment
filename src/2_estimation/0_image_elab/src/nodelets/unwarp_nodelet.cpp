#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <unwarp_handle.hpp>

namespace image_elab
{

	class UnwarpNodelet : public nodelet::Nodelet
	{
	public:
		UnwarpNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getNodeHandle();
			handle_.onInit(nh);
	  	}

	  	image_proc::UnwarpHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(image_elab::UnwarpNodelet, nodelet::Nodelet)
}