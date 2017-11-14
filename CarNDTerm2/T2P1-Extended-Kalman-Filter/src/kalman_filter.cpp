#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);
  P_ = MatrixXd(4, 4);
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
  I_ = MatrixXd::Identity(x_.size(), x_.size());
  H_laser_ = MatrixXd(2, 4);
  H_radar_ = MatrixXd(3, 4);
  
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init() {
  // state vector 
  x_ << 1, 1, 1, 1;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,       0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,     0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  H_radar_ << 1, 1, 0, 0,
              1, 1, 0, 0,
              1, 1, 1, 1; 

  //transition transition matrix F_
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // covariance matrix P_
  P_ << 10, 0, 0, 0,
        0, 10, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  
  noise_ax = 9;
  noise_ay = 9;
}

void KalmanFilter::Predict(float dt) {
  /**
  TODO:
    * predict the state
  */
  if(dt < 0.01) {
    cout << "warn: update dt is too small " << dt << endl; 
    return;    
  }
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  //Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
          0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
          dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
          0, dt_3/2*noise_ay, 0, dt_2*noise_ay; 

  x_ = F_*x_ /* + *u_*/; // u_ is zero meaned variable
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::LaserUpdate(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd H_ = H_laser_;
  MatrixXd R_ = R_laser_;

  VectorXd y  = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S  = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}


static const float sPI = 3.14159265;

void KalmanFilter::RadarUpdate(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd y; 
  VectorXd z_pred (3);
  float rho_pred = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi_pred = atan2(x_(1), x_(0));
  float rho_dot_pred = 0.0;
  if( rho_pred < 0.001)
    rho_pred = 0.001;

  // handle divided_by_zero error
  rho_dot_pred = (x_(0)*x_(2) + x_(1)*x_(3)) / rho_pred;
  z_pred << rho_pred, phi_pred, rho_dot_pred;

  y = z - z_pred;
  // normalize the angle error to range [-pi, pi]
  double *phi_error = &y(1);
  if(fabs(*phi_error) > sPI) {

    while(*phi_error > sPI) {
      *phi_error -= 2*sPI;
    }	

    while(*phi_error < -sPI) {
      *phi_error += 2*sPI;
    }	
  }

  // Update Hj
  MatrixXd H_ = tools_.CalculateJacobian(x_);;
  MatrixXd R_ = R_radar_;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}
