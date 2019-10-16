#!/bin/bash

CATKIN_SHELL=bash

printf "* RoboticsEnvironment/environment.sh\n\n"

# Display Applied Robotics logo
printf "       \e[32m++++++ Applied Robotics  ++++++\n"
printf "       |\e[34m                             \e[32m|\n"
printf "       |\e[34m           \_\               \e[32m|\n"
printf "       |\e[34m          (_**)              \e[32m|\n"
printf "       |\e[34m         __) #_              \e[32m|\n"
printf "       |\e[34m        ( )...()             \e[32m|\n"
printf "       |\e[34m        || | |I|             \e[32m|\n"
printf "       |\e[34m        || | |()__/          \e[32m|\n"
printf "       |\e[34m        /\(___)              \e[32m|\n"
printf "       |\e[34m       _-\"\"\"\"\"\"\"-_\"\"-_       \e[32m|\n"
printf "       |\e[34m       -,,,,,,,,- ,,-        \e[32m|\n"
printf "       \e[33mTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT\n\n\e[0m"


if [ -z "$AR_CATKIN_ROOT" ]
then
  export AR_CATKIN_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)
  source ${AR_CATKIN_ROOT}/aliases

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
      printf "\t\t"
      read -p "Want to build it now? [y/n]" -n 1 -r
      printf "\n"
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

printf "\t*\e[32m chrpath converting rpath to runpath\e[0m\n"  
# Convert rpath to runpath
chrpath -c $AR_CATKIN_ROOT/devel/lib/libimage_elab_LIBRARY.so&> /dev/null
chrpath -c $AR_CATKIN_ROOT/devel/lib/libimage_elab_nodelet.so&> /dev/null
chrpath -c $AR_CATKIN_ROOT/devel/lib/image_elab/*&> /dev/null
chrpath -c $AR_CATKIN_ROOT/devel/lib/planning/*&> /dev/null
