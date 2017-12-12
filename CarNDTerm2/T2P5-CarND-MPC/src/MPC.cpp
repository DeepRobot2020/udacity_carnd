#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;
using namespace std;

class FG_eval
{
  public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars)
    {
        // Implementing MPC below
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // Cost for CTE, psi error and velocity
        for (uint32_t  t = 0; t < MPC_CONFIG::N; t++)
        {
            fg[0] += 2000 * CppAD::pow(vars[MPC_CONFIG::cte_start + t] - MPC_CONFIG::ref_cte, 2);
            fg[0] += 2000 * CppAD::pow(vars[MPC_CONFIG::epsi_start + t] - MPC_CONFIG::ref_epsi, 2);
            fg[0] += 1 * CppAD::pow(vars[MPC_CONFIG::v_start + t] - MPC_CONFIG::ref_v, 2);
        }

        // Minimize the use of actuators.
        for (uint32_t  t = 0; t < MPC_CONFIG::N - 1; t++)
        {
            fg[0] += 10 * CppAD::pow(vars[MPC_CONFIG::delta_start + t], 2);
            fg[0] += 10 * CppAD::pow(vars[MPC_CONFIG::a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (uint32_t  t = 0; t < MPC_CONFIG::N - 2; t++)
        {
            fg[0] += 100 * CppAD::pow(vars[MPC_CONFIG::delta_start + t + 1] - vars[MPC_CONFIG::delta_start + t], 2);
            fg[0] += 10 * CppAD::pow(vars[MPC_CONFIG::a_start + t + 1] - vars[MPC_CONFIG::a_start + t], 2);
        }

        // Setup Model Constraints

        fg[1 + MPC_CONFIG::x_start] = vars[MPC_CONFIG::x_start];
        fg[1 + MPC_CONFIG::y_start] = vars[MPC_CONFIG::y_start];
        fg[1 + MPC_CONFIG::psi_start] = vars[MPC_CONFIG::psi_start];
        fg[1 + MPC_CONFIG::v_start] = vars[MPC_CONFIG::v_start];
        fg[1 + MPC_CONFIG::cte_start] = vars[MPC_CONFIG::cte_start];
        fg[1 + MPC_CONFIG::epsi_start] = vars[MPC_CONFIG::epsi_start];

        // The rest of the constraints
        for (uint32_t t = 1; t < MPC_CONFIG::N; t++)
        {
            // The state at time t+1 .
            AD<double> x1 = vars[MPC_CONFIG::x_start + t];
            AD<double> y1 = vars[MPC_CONFIG::y_start + t];
            AD<double> psi1 = vars[MPC_CONFIG::psi_start + t];
            AD<double> v1 = vars[MPC_CONFIG::v_start + t];
            AD<double> cte1 = vars[MPC_CONFIG::cte_start + t];
            AD<double> epsi1 = vars[MPC_CONFIG::epsi_start + t];

            // The state at time t.
            AD<double> x0 = vars[MPC_CONFIG::x_start + t - 1];
            AD<double> y0 = vars[MPC_CONFIG::y_start + t - 1];
            AD<double> psi0 = vars[MPC_CONFIG::psi_start + t - 1];
            AD<double> v0 = vars[MPC_CONFIG::v_start + t - 1];
            AD<double> cte0 = vars[MPC_CONFIG::cte_start + t - 1];
            AD<double> epsi0 = vars[MPC_CONFIG::epsi_start + t - 1];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[MPC_CONFIG::delta_start + t - 1];
            AD<double> a0 = vars[MPC_CONFIG::a_start + t - 1];

            AD<double> f0 = coeffs[0] + coeffs[1] * x0 +
                            coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
            AD<double> psi_des0 = CppAD::atan(coeffs[1] +
                                              2 * coeffs[2] * x0 +
                                              3 * coeffs[3] * pow(x0, 2));

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[1 + MPC_CONFIG::x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * MPC_CONFIG::dt);
            fg[1 + MPC_CONFIG::y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * MPC_CONFIG::dt);
            fg[1 + MPC_CONFIG::psi_start + t] = psi1 - (psi0 - v0 * delta0 / MPC_CONFIG::Lf * MPC_CONFIG::dt);
            fg[1 + MPC_CONFIG::v_start + t] = v1 - (v0 + a0 * MPC_CONFIG::dt);

            fg[1 + MPC_CONFIG::cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * MPC_CONFIG::dt));
            fg[1 + MPC_CONFIG::epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / MPC_CONFIG::Lf * MPC_CONFIG::dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs)
{
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = x0[0];
    double y = x0[1];
    double psi = x0[2];
    double v = x0[3];
    double cte = x0[4];
    double epsi = x0[5];

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    const size_t n_states = x0.size();
    const size_t n_actuators = 2;
    // For N states, there is N-1 actuators
    const size_t n_vars = n_states * MPC_CONFIG::N + n_actuators * (MPC_CONFIG::N - 1);

    // Set the number of constraints
    const size_t n_constraints = MPC_CONFIG::N * n_states;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (uint32_t  i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }
    // Set the initial variable values
    vars[MPC_CONFIG::x_start] = x;
    vars[MPC_CONFIG::y_start] = y;
    vars[MPC_CONFIG::psi_start] = psi;
    vars[MPC_CONFIG::v_start] = v;
    vars[MPC_CONFIG::cte_start] = cte;
    vars[MPC_CONFIG::epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // TODO: Set lower and upper limits for variables.

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (uint32_t i = 0; i < MPC_CONFIG::delta_start; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (uint32_t  i = MPC_CONFIG::delta_start; i < MPC_CONFIG::a_start; i++)
    {
        vars_lowerbound[i] = -0.436332; //* MPC_CONFIG::Lf;
        vars_upperbound[i] = 0.436332;  // * MPC_CONFIG::Lf;
        // vars_lowerbound[i] = -1.0;
        // vars_upperbound[i] = 1.0;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (uint32_t  i = MPC_CONFIG::a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (uint32_t  i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[MPC_CONFIG::x_start] = x;
    constraints_lowerbound[MPC_CONFIG::y_start] = y;
    constraints_lowerbound[MPC_CONFIG::psi_start] = psi;
    constraints_lowerbound[MPC_CONFIG::v_start] = v;
    constraints_lowerbound[MPC_CONFIG::cte_start] = cte;
    constraints_lowerbound[MPC_CONFIG::epsi_start] = epsi;

    constraints_upperbound[MPC_CONFIG::x_start] = x;
    constraints_upperbound[MPC_CONFIG::y_start] = y;
    constraints_upperbound[MPC_CONFIG::psi_start] = psi;
    constraints_upperbound[MPC_CONFIG::v_start] = v;
    constraints_upperbound[MPC_CONFIG::cte_start] = cte;
    constraints_upperbound[MPC_CONFIG::epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more pruint32_t  information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.25\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    std::vector<double> results;
    results.push_back(solution.x[MPC_CONFIG::delta_start]);
    results.push_back(solution.x[MPC_CONFIG::a_start]);

    for (uint32_t  i = 0; i < MPC_CONFIG::N; i++)
    {
        results.push_back(solution.x[MPC_CONFIG::x_start + i]);
        results.push_back(solution.x[MPC_CONFIG::y_start + i]);
    }
    return results;
}
