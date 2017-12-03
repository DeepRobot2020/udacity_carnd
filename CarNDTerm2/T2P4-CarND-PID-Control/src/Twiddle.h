#include <limits>
#include <stdint.h>
#include <vector>

class Twiddle
{
    enum VisitStatus
    {
        visting_current = 0,
        visting_postive_side,
        visting_negative_side,
        all_visited
    };

  public:
    ~Twiddle();
    Twiddle();
    /*
    * Log training error with current parameters
    */
    void UpdateCurrentParameterError(double error);
    /*
    * Get current training parameters
    */
    std::vector<double> GetParameters();
    /*
    * Whether twiddle training is complete
    */
    bool IsCompleted();

  private:
    /*
    * Current training parameter index
    * range [0, 2]
    */
    uint8_t paramter_index = 0;
    /*
    * Status of current training parameter
    *   visting_current: the very first training
    *   visting_postive_side: traiing with parameters slightly larger
    *    visting_negative_side: traiing with parameters slightly smaller
    */
    uint8_t visit_status;

    double best_error;

    /*
    * PID parameters:
    * Note: this is the optimal value after my twiddle training with 
    * a couple of full tracks
    */
    std::vector<double> p = {0.2, 1e-6, 1.543};
    std::vector<double> dp = {0.1, 1e-6, 0.08};

    const std::vector<double> p_min = {0.01, 0, 0.01};
    const std::vector<double> p_max = {4, 0.01, 4};
};
