#ifndef PATH_H
#define PATH_H

#include <vector>
#include <cmath>
#include <cstddef>
// A configuration of the robot along the path, represented by x, y, orientation and curvature
struct Pose 
{
  float s, x, y, theta, kappa;

  Pose(float s, float x, float y, float theta, float kappa):
    s(s), x(x), y(y), theta(theta), kappa(kappa)
  {}

  Pose(): 
    Pose(0, 0, 0, 0, 0)
  {}

  float distance(float _x, float _y)
  {
    return std::hypot(x-_x, y-_y);
  }
};

// A sequence of sampled robot configurations composing a (discretization of the) path
struct Path 
{
  std::vector<Pose> points;
  
  Path(std::vector<Pose> const & points):
    points(points)
  {}

  Path()
  {}
  
  bool empty() { return points.empty(); }
  size_t size() { return points.size(); }
  void setPoints(const std::vector<Pose>& points) { this->points = points; }
};

struct Point 
{
  float x, y;

  Point(float x, float y):
    x(x), y(y)
  {}

  Point(): 
    Point(0, 0)
  {}

};


typedef std::vector<Point> Polygon;


#endif 
