#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <control_handle.hpp>

namespace control
{

	class ControlNodelet : public nodelet::Nodelet
	{
	public:
		ControlNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getNodeHandle();
			handle_.onInit(nh);
	  	}

	  	ControlHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(control::ControlNodelet, nodelet::Nodelet)
}