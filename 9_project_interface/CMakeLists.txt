cmake_minimum_required(VERSION 2.8.3)
project(project_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES student professor
   CATKIN_DEPENDS 
   DEPENDS opencv
)

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
)

## CREATE SHARED LIBRARY
add_library(student SHARED
   src/dummy_interface.cpp
)

target_link_libraries(student
  ${catkin_LIBRARIES}  
)

add_library(professor SHARED
   src/professor_interface.cpp
)

target_link_libraries(professor
  ${catkin_LIBRARIES}  
)
