#include "path_following.hpp"

static inline double rangeSymm(double ang) 
{
  while (ang <= - M_PI) ang += 2 * M_PI;
  while (ang >  M_PI) ang -= 2 * M_PI;
  return ang;
}

size_t PathFollowing::getFrenetPoint(double x, double y, double theta, Pose& pose, double win_size, size_t idx)
{
  if (path.empty() || idx>=path.size()) 
  {
    throw std::out_of_range("Accessing empty path");
  }

  std::vector<Pose> & points = path.points;

  double dmin = points[idx].distance(x, y);
  double s = points[idx].s;
  int imin = idx;

  for (int i = idx-1; i>=0 && s-points[i].s<win_size; --i)
  {
    double cmin = points[i].distance(x, y);
    if (cmin < dmin)
    {
      imin = i;
      dmin = cmin;
    }
  }

  for (int i = idx+1; i<points.size() && points[i].s-s<win_size; ++i)
  {
    double cmin = points[i].distance(x, y);
    if (cmin < dmin)
    {
      imin = i;
      dmin = cmin;
    }
  }

  pose = points[imin];
  return imin;
}

// Compute the curvature to follow to reach the frenet point
double PathFollowing::computeControl(const Pose& fp, double x, double y, double theta)
{
  
  // control constants
  const double k = 1.5; 
  const double l0 = 0.1;
  
  // Estract useful values
  double delta_x  = fp.x - x;
  double delta_y  = fp.y - y;
  double thetaDes = rangeSymm(fp.theta-theta);
  double theta_f =  rangeSymm(fp.theta); // absolute angle of frenet point
  
  double l = -std::cos(theta_f+M_PI/2.)*delta_x - std::sin(theta_f+M_PI/2.)*delta_y;
  double c = fp.kappa; // curvature in the point
            
  double thetaTilde = -thetaDes;

  double tanhl = std::tanh(l/l0);
  double delta = -M_PI/2.*tanhl;
            
  double Ddelta_dl = -M_PI/2.*(1.-tanhl*tanhl)/l0;
  double gamma = c*std::cos(thetaTilde)/(1-c*l) + std::sin(thetaTilde)*Ddelta_dl;
            
  double tmp = rangeSymm(thetaTilde-delta);
  double rho = gamma - k*tmp;
  return rho;
}


double PathFollowing::computeControl(double x, double y, double theta, double& s)
{
  Pose fp;
  if (idx < 0) 
  {
    idx = getFrenetPoint(x, y, theta, fp);
  }
  else 
  {
    idx = getFrenetPoint(x, y, theta, fp, 0.2, idx);
  }

  s = fp.s;

  if (idx == path.size()-1) eop = true;

  return computeControl(fp, x, y, theta);
}

