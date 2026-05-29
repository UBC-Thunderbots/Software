#pragma once

#include <algorithm>
#include <cassert>
#include <optional>

/**
 * A PID controller is used to calculate corrections based on error values over
 * time as the difference between a desired value and the actual measured value.
 * This PID controller also limits integral windup using a max_integral value.
 *
 * Resources:
 * - https://raw.org/book/control-theory/introduction-to-pid-controllers/
 * - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-reset-windup/
 */
template <typename T>
class PidController
{
   public:
    /**
     * Constructs a new PID controller.
     *
     * @pre max_integral must be >= 0.0
     *
     * @param k_p The proportional gain.
     * @param k_i The integral gain.
     * @param k_d The derivative gain.
     * @param max_integral The maximum absolute value that the integrator term can
     * accumulate to.
     **/
    PidController(T k_p, T k_i, T k_d, T max_integral)
        : k_p_(k_p), k_i_(k_i), k_d_(k_d), max_integral_(max_integral)
    {
        assert(max_integral >= T(0.0));
    }

    /**
     * Given an error, returns the control effort to minimize it.
     *
     * @param error The amount of error between desired and actual output
     * @param delta_time The time passed since last step, for calculating the integrator
     * and derivative.
     **/
    T step(T error, T delta_time = T(1.0))
    {
        // If sign of error swaps, reset integrator
        if (last_error_.has_value() && (last_error_.value() * error < T(0.0)))
        {
            integral_ = T(0.0);
        }

        integral_ =
            std::clamp(integral_ + error * delta_time, -max_integral_, max_integral_);

        const T derivative = (error - last_error_.value_or(error)) / delta_time;

        last_error_ = error;

        return error * k_p_ + integral_ * k_i_ + derivative * k_d_;
    }

    /**
     * Resets the integrator and clears the last error used for derivative calculation.
     **/
    void reset()
    {
        integral_   = T(0.0);
        last_error_ = std::nullopt;
    }

   private:
    T k_p_;
    T k_i_;
    T k_d_;
    T max_integral_;

    T integral_                  = T(0.0);
    std::optional<T> last_error_ = std::nullopt;
};
