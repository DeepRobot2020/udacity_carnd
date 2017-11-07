#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // We need initialize the UKF on receiving the first measurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 1, 1, 1, 1, 1;
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ = MatrixXd::Identity(5, 5);

  R1_ = MatrixXd(n_l_, n_l_);
  R1_ <<  std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;

  R2_ = MatrixXd(n_z_,n_z_);
  R2_ <<  std_radr_*std_radr_, 0, 0,
         0, std_radphi_*std_radphi_, 0,
         0, 0, std_radrd_*std_radrd_;

 

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  // State dimension 
  n_x_ = x_.size();

  // Augmented state dimension
  n_aug_ = n_x_ + 2;
  // Sigma point spreading parameter
  lambda_= 3 - n_aug_;
  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0);

  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(0);

  // Initialize the weights for sigmna points
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i< 2*n_aug_+1; i++) {  
    weights_(i) = 0.5/(n_aug_+lambda_);
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /** Convert radar from polar to cartesian coordinates and initialize state.
       */
      double rho     = meas_package.raw_measurements_(0);
      double phi     = meas_package.raw_measurements_(1);
     
      x_(0) = rho * cos ( phi );
      x_(1) = rho * sin ( phi );
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);  
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    std::cout << "UKF is initialized!" << std::endl;
    return;
  }
  
  std::cout << "Prediction starts" << std::endl;

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(dt);
  std::cout << "Prediction done" << std::endl;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    std::cout << "UpdateLidar starts" << std::endl;
    // Radar update
    UpdateLidar(meas_package);
    std::cout << "UpdateLidar is done" << std::endl;
  } else {
    std::cout << "UpdateRadar starts" << std::endl;
    // Laser updates
    UpdateRadar(meas_package);
    std::cout << "UpdateRadar is done" << std::endl;
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++) {
    //extract values for better readability
    double p_x  = Xsig_pred_(0,i);
    double p_y  = Xsig_pred_(1,i);
    double v    = Xsig_pred_(2,i);
    double yaw  = Xsig_pred_(3,i);
    double yawd = Xsig_pred_(4,i);
    double nu_a = Xsig_pred_(5,i);
    double nu_yawdd = Xsig_pred_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * (sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  //predicted state mean
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  //predicted state covariance matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  MatrixXd Zsig = MatrixXd(n_l_, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) { //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    // measurement model
    Zsig(0, i) = p_x; //px
    Zsig(1, i) = p_y; //py
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_l_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_l_, n_l_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 
    //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
    
  //add measurement noise covariance matrix
  S = S + R1_;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_l_);
  VectorXd z = meas_package.raw_measurements_;
    
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //transform sigma points into measurement space
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1  = cos(yaw)*v;
    double v2  = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  S = S + R2_;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  VectorXd z = meas_package.raw_measurements_;
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}