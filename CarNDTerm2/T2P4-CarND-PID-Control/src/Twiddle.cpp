#include "Twiddle.h"
#include <limits>
#include <numeric>
#include <iostream>

Twiddle::Twiddle()
{
    paramter_index = 0;
    visit_status = visting_current;
    best_error = std::numeric_limits<double>::max();
}
Twiddle::~Twiddle() {}

void Twiddle::UpdateCurrentParameterError(double error)
{
    std::cout << "===================== " << std::endl;
    std::cout << "UpdateCurrentParameterError: " << std::endl;
    std::cout << "visit_status: " << (int)(visit_status) << std::endl;
    std::cout << "paramter_index: " << (int)paramter_index << std::endl;
    std::cout << "error: " << error << std::endl;
    std::cout << "best_error: " << best_error << std::endl;
    std::cout << "Kp: " << p[0] << std::endl;
    std::cout << "Ki: " << p[1] << std::endl;
    std::cout << "Kd: " << p[2] << std::endl;
    std::cout << "===================== " << std::endl;

    if (visit_status == visting_current)
    {
        best_error = error;
        p[paramter_index] += dp[paramter_index];
        visit_status = visting_postive_side;
        return;
    }

    switch (visit_status)
    {
    case visting_postive_side:
        if (error < best_error) // There was an improvement
        {
            best_error = error;
            dp[paramter_index] *= 1.1;
            visit_status = all_visited;
        }
        else // There was no improvement
        {
            p[paramter_index] -= 2 * dp[paramter_index];
            visit_status = visting_negative_side;
        }
        break;

    case visting_negative_side:
        visit_status = all_visited;
        if (error < best_error) // There was an improvement
        {
            best_error = error;
            dp[paramter_index] *= 1.05;
        }
        else // There was no improvement
        {
            p[paramter_index] += dp[paramter_index];
            dp[paramter_index] *= 0.9;
        }
        break;
    default:
        break;
    }

    if (visit_status == all_visited)
    {
        paramter_index += 1;
        paramter_index = paramter_index % 3;
        p[paramter_index] += dp[paramter_index];
        visit_status = visting_postive_side;
    }
}

std::vector<double> Twiddle::GetParameters()
{
    std::cout << "********************** " << std::endl;
    std::cout << "GetParameters: " << std::endl;
    std::cout << "visit_status: " << (int)(visit_status) << std::endl;
    std::cout << "paramter_index: " << (int)paramter_index << std::endl;
    std::cout << "Kp: " << p[0] << std::endl;
    std::cout << "Ki: " << p[1] << std::endl;
    std::cout << "Kd: " << p[2] << std::endl;

    std::cout << "dp_p: " << dp[0] << std::endl;
    std::cout << "dp_i: " << dp[1] << std::endl;
    std::cout << "dp_d: " << dp[2] << std::endl;
    std::cout << "********************** " << std::endl;
    return p;
}

bool Twiddle::IsCompleted()
{
    double sum_dp = std::accumulate(dp.begin(), dp.end(), 0.0);
    bool dp_is_good = true;
    for(int i = 0; i < 3; i++)
    {
        dp_is_good &= (dp[i] > p_min[i] && dp[i] < p_max[i]);
    }
    // std::cout << "sum_dp " << sum_dp << std::endl;
    // std::cout << "best_error " << best_error << std::endl;
    // std::cout << "dp_is_good " << dp_is_good << std::endl;

    return sum_dp < 0.01 && 
         best_error < 1.0 &&
         dp_is_good;
}
