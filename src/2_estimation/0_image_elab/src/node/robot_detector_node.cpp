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
#include "robot_detector_handle.hpp"

using namespace image_proc;

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_detector_handle_node");
  ros::NodeHandle nodeHandle("~");

  RobotDetectorHandle detector;
  detector.onInit(nodeHandle);

  ros::spin();
  
  return 0;
}
