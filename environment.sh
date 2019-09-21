#!/bin/bash

CATKIN_SHELL=bash

printf "* RoboticsEnvironment/environment.sh\n"
if [ -z "$AR_CATKIN_ROOT" ]
then
  export AR_CATKIN_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)
  source ${AR_CATKIN_ROOT}/aliases

  # Display Applied Robotics logo
  AR_logo

  # check whether devel folder exists
  printf "\t* Loading ROS environment\n"
  if [ -f "${AR_CATKIN_ROOT}/devel/setup.bash" ]; then
      # source setup.sh from same directory as this file
      source "${AR_CATKIN_ROOT}/devel/setup.bash"
      printf "\t\t* \e[32m DONE!\n\e[0m"
  else
      source "/opt/ros/kinetic/setup.bash"
      printf "\t\e[31mYou need to build first before you can source\n\e[0m"
      printf "\t\e[33mRun 'catkin build' in ${AR_CATKIN_ROOT} directory\n\e[0m"
      read -p "\t\tWant to build it now? [y/n]" -n 1 -r
      if [[ $REPLY =~ ^[Yy]$ ]]
      then
        catkin clean --yes
        catkin build
        source "${AR_CATKIN_ROOT}/devel/setup.bash"
      fi
  fi
else
  printf "\t*\e[33m Seems that you already source this environment.\e[0m\n"  
fi
