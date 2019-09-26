/*!****************************************************************************
 *    \file    kalman_filter.hpp
 *    \brief   Class that implement a basic kalman filter (code taken from Udacity) it is a template on the dimention of the state
 *    \version 1.0
 *    \date    2017
 *****************************************************************************/
/// @file   Algorithm.hpp
/// @brief  Class that implement a basic kalman filter (code taken from Udacity)
/// @author Valerio Magnago, Paolo Bevilacqua
///

#include "extended_kalman_filter.hpp"


static inline double rangeSymm(double ang) 
{
  while (ang <= - M_PI) ang += 2 * M_PI;
  while (ang >  M_PI) ang -= 2 * M_PI;
  return ang;
}


/**
 * Constructor
  */
ExtendedKalmanFilter::ExtendedKalmanFilter(): 
  initialized(false)
{};  // Nothing to inizialize


void ExtendedKalmanFilter::reset(){
  initialized = false;
}

void ExtendedKalmanFilter::predict(Vec2& u, Matrix2& Q)
{
  if(!initialized) return;
  
  // predict next state using Kalman filter.
  double ds  = u(0);
  double dth = u(1);

  double x_, y_, theta_;
  Matrix3 F = Matrix3::Identity();
  
  if (std::abs(dth)<1e-2)
  {
    x_ = x(0) + ds*std::cos(x(2) + dth/2);
    y_ = x(1) + ds*std::sin(x(2) + dth/2);   
    theta_ = x(2) + dth;

    F(0,2) = -ds*std::sin(x(2)  + dth/2);    
    F(1,2) =  ds*std::cos(x(2)  + dth/2);
  }
  else
  {
    double R = ds/dth;
    x_ = x(0) + (std::sin(dth+x(2))-std::sin(x(2)))*R;
    y_ = x(1) - (std::cos(dth+x(2))-std::cos(x(2)))*R;            
    theta_ = x(2) + dth;

    F(0,2) = R*(std::cos(x(2)+dth)-std::cos(x(2)));
    
    F(1,2) = R*(std::sin(x(2)+dth)-std::sin(x(2)));    
  }
 
  Matrix32 V = Matrix32::Zero();
  V(0,0) = std::cos(x(2));
  V(1,0) = std::sin(x(2));
  V(2,1) = 1;
 
  P = F*P*(F.transpose()) + V*Q*V.transpose(); 

  x(0) = x_; x(1) = y_; x(2) = theta_;
}

void ExtendedKalmanFilter::updateGPS(const Vec3& z, const Matrix3& R)
{
  if(!initialized){
    x = z;
    P = R;
    initialized = true;
    return;
  }
  /**
  * update the state by using Kalman Filter equations
  */    
  const Matrix3 H = Matrix3::Identity();
  Vec3 y = z - H * x;
  const Matrix3 Ht = H.transpose();
  const Matrix3 S = H * P * Ht + R;
  const Matrix3 Si = S.inverse();
  const Matrix3 K =  P * Ht * Si;
  const Matrix3 I = Matrix3::Identity();

  y(2) = rangeSymm(y(2));
  x = x + (K * y);  // new state
  P = (I - K * H) * P; // new covariance  
}

bool ExtendedKalmanFilter::isLocalized()
{
  return initialized;
}
