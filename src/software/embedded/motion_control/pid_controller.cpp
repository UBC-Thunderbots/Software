#include "software/embedded/motion_control/pid_controller.h"

#include <algorithm>
#include <stdexcept>

PidController::PidController(double k_p, double k_i, double k_d, double max_integral)
    : k_p(k_p), k_i(k_i), k_d(k_d), max_integral(max_integral)
{
    if (max_integral < 0.0)
    {
        throw std::invalid_argument("PidController max_integral must be >= 0.0");
    }
};

double PidController::step(double error, double delta_time)
{
    integral_ = std::clamp(integral_ + error * delta_time, -max_integral, max_integral);

    const double derivative = (error - last_error_.value_or(error)) / delta_time;

    last_error_ = error;

    return error * k_p + integral_ * k_i + derivative * k_d;
}

void PidController::reset()
{
    integral_   = 0.0;
    last_error_ = std::nullopt;
}
