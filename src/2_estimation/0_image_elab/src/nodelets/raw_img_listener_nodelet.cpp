#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <generic_handle.hpp>

namespace image_elab
{

	class RawImageListenerNodelet : public nodelet::Nodelet
	{
	public:
		RawImageListenerNodelet()
	  	{}

	private:
		virtual void onInit()
		{
			std::string default_implementation_field = "/default_implementation/image_raw_clbk";
  			std::string camera_subscriber_topic_name = "/camera/rgb/image_raw";

		    ros::NodeHandle& nh = getMTNodeHandle();
			handle_.onInit(nh, default_implementation_field, camera_subscriber_topic_name);
			  			
	  	}

	  	image_proc::GenericHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(image_elab::RawImageListenerNodelet, nodelet::Nodelet)
}