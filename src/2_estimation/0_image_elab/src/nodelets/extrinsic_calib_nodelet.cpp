#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <extrinsic_handle.hpp>

namespace image_elab
{

	class ExtrinsicCalibNodelet : public nodelet::Nodelet
	{
	public:
		ExtrinsicCalibNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getMTNodeHandle();
			handle_.onInit(nh);
	  	}

	  	image_proc::ExtrinsicHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(image_elab::ExtrinsicCalibNodelet, nodelet::Nodelet)
}