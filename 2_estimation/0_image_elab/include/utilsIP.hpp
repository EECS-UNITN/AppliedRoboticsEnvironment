#pragma once

#include <opencv2/opencv.hpp>
#include "geometry_msgs/Polygon.h"
#include "utils.hpp"

namespace image_proc {

geometry_msgs::Polygon createPolygon(const Polygon & poly_2D);        

}