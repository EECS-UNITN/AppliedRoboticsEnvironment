#include "utils.hpp"

namespace image_proc {

typedef std::vector<cv::Point2f> Polygon;

geometry_msgs::Polygon createPolygon(const Polygon & poly_2D){

    geometry_msgs::Polygon poly_3D;
    for (const auto & pt: poly_2D){ 
        geometry_msgs::Point32 pt_3D;
        pt_3D.x = pt.x;
        pt_3D.y = pt.y;
        pt_3D.z = 0.;
        poly_3D.points.push_back(pt_3D);
    }
    return poly_3D;
}
        

}