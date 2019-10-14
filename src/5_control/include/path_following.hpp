#ifndef PATH_FOLLOWING_H
#define PATH_FOLLOWING_H

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

#include "utils.hpp"

class PathFollowing 
{
public:
  void setPath(const Path& path) { this->path = path; idx = -1; eop = false; }
  double computeControl(double x, double y, double theta, double& fp_s, double& fp_x, double& fp_y);
  bool endOfPath() { return eop; }

private:
  size_t getFrenetPoint(double x, double y, double theta, 
                        Pose& pose, 
                        double win_size = std::numeric_limits<double>::infinity(), 
                        size_t idx = 0);
  double computeControl(const Pose& fp, double x, double y, double theta);
  Path path;
  int idx;
  bool eop = true;
};


#endif 