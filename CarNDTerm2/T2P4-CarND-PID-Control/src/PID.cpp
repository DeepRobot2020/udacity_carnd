#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    v_cte = 0.0;
    v_speed = 0.0;
    v_angle = 0.0;
    this->Kp = 2;
    this->Ki = 0; 
    this->Kd = 0;
}

void PID::UpdateVehicle(double cte, double speed, double angle) {
    v_cte = cte;
    v_speed = speed;
    v_angle = angle;
}

double PID::CalcualteSteeringAngle() {
    double steering_angle = 0;
    steering_angle = -Kp * p_error - Ki * i_error - Kd * d_error;
    return steering_angle;
}


double PID::TotalError() {

}

