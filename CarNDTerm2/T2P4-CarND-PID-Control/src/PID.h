#ifndef PID_H
#define PID_H

class PID
{
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  /*
  * Speed Error: for maintaining constant speed
  */
  double s_error;

  double error_squre_sum;

  double v_cte;
  int n_steps;
  int n_errors;
  bool is_initialized;
  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);
  /*
  * Calculate Steering Angle.
  */
  double CalcualteSteeringAngle();
  /*
  * Calculate throttling value to maintain constant speed.
  */
  double CalcualteThrottling();

  /*
  * Update the PID error variables given cross track error.
  * Also updated speed error
  */
  void UpdateErrors(double cte, double se);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Reset PID controller.
  */
  void Reset();
};

#endif /* PID_H */
