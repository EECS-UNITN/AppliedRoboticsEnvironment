#pragma once

#include <opencv2/opencv.hpp>
#include "geometry_msgs/Polygon.h"

namespace image_proc {

typedef std::vector<cv::Point2f> Polygon;

geometry_msgs::Polygon createPolygon(const Polygon & poly_2D);        

}