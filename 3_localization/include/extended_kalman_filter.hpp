/*!****************************************************************************
 *    \file    kalman_filter.hpp
 *    \brief   Class that implement a basic kalman filter (code taken from Udacity) it is a template on the dimention of the state
 *    \version 1.0
 *    \date    2017
 *****************************************************************************/
/// @file   Algorithm.hpp
/// @brief  Class that implement a basic kalman filter (code taken from Udacity)
/// @author Valerio Magnago
///

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
#include <vector>

typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 3, 2> Matrix32;
typedef Eigen::Matrix<double, 2, 2> Matrix2;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;

class ExtendedKalmanFilter {
public:
	bool initialized;

	Vec3 x;
	Matrix3 P;
	double timestamp;

	/**
	 * Constructor
	 */
	ExtendedKalmanFilter();

	void reset();
	
	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model. x_ = F_ * x_ + G*u_
	 * @param u is the input
	 * @param Q is the covariance of the input
	 */
	void predict(Vec2& u, Matrix2& Q);

	/**
	 * Updates the state by using standard linear Kalman Filter update equations	 
	 * @param z The measurement at k+1	 
	 * @param R is the measurement covariance matrix
	 */
	void updateGPS(const Vec3& z, const Matrix3& R);

	bool isLocalized();

};


#endif /* KALMAN_FILTER_H_ */

