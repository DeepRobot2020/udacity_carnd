#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // laser measurement matrix
  Eigen::MatrixXd H_laser_;

  // rader measurement matrix
  Eigen::MatrixXd H_radar_;
  
  // laser measurement covariance matrix
  Eigen::MatrixXd R_laser_;

  // rader measurement covariance matrix
  Eigen::MatrixXd R_radar_;

  // Identity matrix 
  Eigen::MatrixXd I_;

    
  float noise_ax;
  float noise_ay;

  Tools tools_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   */
  void Init();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(float dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void LaserUpdate(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void RadarUpdate(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
