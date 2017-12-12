#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

namespace MPC_CONFIG
{
    // Set the timestep length and duration
    static constexpr size_t N = 10;
    static constexpr double dt = 0.1;

    // the solver takes all the state variables and actuator
    // variables in a singular vector. thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    static constexpr size_t x_start     = 0;
    static constexpr size_t y_start     = x_start     + N;
    static constexpr size_t psi_start   = y_start     + N;
    static constexpr size_t v_start     = psi_start   + N;
    static constexpr size_t cte_start   = v_start     + N;
    static constexpr size_t epsi_start  = cte_start   + N;
    static constexpr size_t delta_start = epsi_start  + N;
    static constexpr size_t a_start     = delta_start + N - 1;
    static constexpr double ref_v       = 100;
    static constexpr double ref_cte     = 0;
    static constexpr double ref_epsi    = 0;

    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius.
    static constexpr double Lf = 2.67;
};

class MPC
{
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state x0 and polynomial coefficients.
    // Return the first actuatotions.
    std::vector<double> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);
private:

};

#endif /* MPC_H */
