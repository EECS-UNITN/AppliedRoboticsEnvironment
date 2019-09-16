cmake_minimum_required(VERSION 2.8.3)
project(camera_calibrator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
)

## CREATE EXECUTABLE LIBRARY
add_executable(camera_calibration
   src/camera_calibration.cpp
)

target_link_libraries(camera_calibration
  ${OpenCV_LIBRARIES}  
)
