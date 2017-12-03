#include "PID.h"

using namespace std;
#include <algorithm>
#include <math.h>
#include <limits>

PID::PID()
{
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    s_error = 0.0;

    v_cte = 0.0;
    is_initialized = false;
}

PID::~PID() {}

void PID::Reset()
{
    is_initialized = false;
}

void PID::Init(double Kp, double Ki, double Kd)
{
    if (is_initialized)
        return;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    n_steps = 0;
    n_errors = 0;
    error_squre_sum = 0.0;
    is_initialized = true;
}

void PID::UpdateErrors(double cte, double se)
{
    p_error = cte;
    i_error += cte;
    d_error = cte - v_cte;
    s_error = se;
    v_cte = cte;

    n_steps += 1;
    // if (n_steps > 100)
    {
        error_squre_sum += cte * cte;
    }
}

double PID::CalcualteSteeringAngle()
{
    double weighted_error = 0.0;
    p_error = Kp * p_error;
    d_error = Kd * d_error;
    i_error = Ki * i_error;
    weighted_error = (p_error + d_error + i_error);

    weighted_error = max(-1.0, weighted_error);
    weighted_error = min(1.0, weighted_error);
    return -weighted_error;
}

double PID::CalcualteThrottling()
{
    double throttle = -0.2 * s_error;
    throttle = min(throttle, 0.5);
    throttle = max(throttle, 0.05);
    return throttle;
}

double PID::TotalError()
{
    return sqrt(error_squre_sum);
}