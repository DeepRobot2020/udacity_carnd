#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double v_cte;
  double v_speed;
  double v_angle;

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
  
  double CalcualteSteeringAngle();


  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateVehicle(double cte, double speed, double angle);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
