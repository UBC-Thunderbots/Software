#include "software/embedded/motion_control/pid_controller.h"

#include <algorithm>
#include <cassert>

PidController::PidController(double k_p, double k_i, double k_d, double max_integral)
    : k_p_(k_p), k_i_(k_i), k_d_(k_d), max_integral_(max_integral)
{
    assert(max_integral >= 0.0);
};

double PidController::step(double error, double delta_time)
{
    integral_ = std::clamp(integral_ + error * delta_time, -max_integral_, max_integral_);

    const double derivative = (error - last_error_.value_or(error)) / delta_time;

    last_error_ = error;

    return error * k_p_ + integral_ * k_i_ + derivative * k_d_;
}

void PidController::reset()
{
    integral_   = 0.0;
    last_error_ = std::nullopt;
}
