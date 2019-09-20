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
#include "localization_handle.hpp"

using namespace localization;

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_node");
  ros::NodeHandle nodeHandle("~");

  LocalizationHandle handle;
  handle.onInit(nodeHandle);

  ros::spin();
  
  return 0;
}
