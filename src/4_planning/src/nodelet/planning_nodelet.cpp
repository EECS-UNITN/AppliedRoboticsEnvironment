#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <planning_handle.hpp>

namespace planning
{

	class PlanningNodelet : public nodelet::Nodelet
	{
	public:
		PlanningNodelet()
	  	{}

	private:
		virtual void onInit()
		{
		    ros::NodeHandle& nh = getMTNodeHandle();
			handle_.onInit(nh);
	  	}

	  	PlanningHandle handle_;
	};

	PLUGINLIB_EXPORT_CLASS(planning::PlanningNodelet, nodelet::Nodelet)
}