/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2019                                                      |
 |                                   .__  __                                |
 |                        __ __  ____ |__|/  |_  ____                       |
 |                       |  |  \/    \|  \   __\/    \                      |
 |                       |  |  /   |  \  ||  | |   |  \                     |
 |                       |____/|___|  /__||__| |___|  /                     |
 |                       .__  __         .__                                |
 |                       |__|/  |______  |  | ___.__.                       |
 |                       |  \   __\__  \ |  |<   |  |                       |
 |                       |  ||  |  / __ \|  |_\___  |                       |
 |                       |__||__| (____  /____/ ____|                       |
 |                                     \/     \/                            |
 |                                                                          |
 |                                                                .         |
 |                                                                          |  
 |      Magnago Valerio                                                     |
 |      Bevilacqua Paolo                                                    |
 |      Dipartimento di Ingegneria e scenza dell'informazione               |
 |      Universita` degli Studi di Trento                                   |
 |      email: valerio.magnago@unitn.it                                     |
 |      email: paolo.bevilacqua@unitn.it                                    |
 |                                                                          |
\*--------------------------------------------------------------------------*/

#include <ros/ros.h>
#include "generic_handle.hpp"

using namespace image_proc;

int main(int argc, char **argv) {
  ros::init(argc, argv, "raw_image_listener");
  ros::NodeHandle nodeHandle("~");

  std::string default_implementation_field = "/default_implementation/image_raw_clbk";
  std::string camera_subscriber_topic_name = "/camera/rgb/image_raw";  
  GenericHandle generic_handle;
  generic_handle.onInit(nodeHandle, default_implementation_field, camera_subscriber_topic_name);

  ros::spin();
  
  return 0;
}
