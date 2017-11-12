#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF
{
public:

  /**
   * Constructor
   */
  UKF() = delete;

  /**
   * Constructor
   */
  UKF(double std_a, double std_yawdd);
  
  /**
   * Destructor
   */
  virtual ~UKF();
  
  /**
   * Return mean state vector X
   */
  VectorXd StateMeanX() const { return x_; };

  /**
   * Return state covariance P
   */
  MatrixXd StateCovarianceP() const { return P_; };

  /**
   * Whether the Laser sensor is used for UKF
   */
  bool IsLaserUsed() const;
    
  /**
   * Whether the Radar sensor is used for UKF
   */
  bool IsRadarUsed() const;

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  virtual double ProcessMeasurement(const MeasurementPackage &meas_package);

protected:
  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  virtual void Prediction(double delta_t);

  /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     * @return the NIS of Lidar sensor 
     */
  virtual double UpdateLidar(const MeasurementPackage &meas_package);

  /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     * @return the NIS of Radar sensor
     */
  virtual double UpdateRadar(const MeasurementPackage &meas_package);

  /**
   * Whether the UKF has been initialized
   */
  virtual bool Initialized() const;

private:
  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Lidar NIS
  double lidar_nis_;
  
  ///* Radar NIS
  double radar_nis_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* augmented sigma points matrix
  MatrixXd Xsig_aug_;

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state covariance matrix
  MatrixXd P_aug_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;
  
  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Number of dimmensions of laser sensor: (px, py)
  const int n_l_ = 2;

  ///* Number of dimmensions of radar sensor: (rho, phi, rho_dot)
  const int n_z_ = 3;

  ///* Laser measurement noise standard deviation position1 in m
  const double std_laspx_ = 0.15;

  ///* Laser measurement noise standard deviation position2 in m
  const double std_laspy_ = 0.15;

  ///* Radar measurement noise standard deviation radius in m
  const double std_radr_ = 0.3;

  ///* Radar measurement noise standard deviation angle in rad
  const double std_radphi_ = 0.03;

  ///* Radar measurement noise standard deviation radius change in m/s
  const double std_radrd_ = 0.3;

  ///* Measurement noise covariance matrix for Radar
  MatrixXd R1_;

  // Measurement noise covariance matrix for Radar
  MatrixXd R2_;

  void CalculateAugmentSigmaPoints();
};

#endif /* UKF_H */
