#pragma once

#include <optional>

class PidController
{
   public:
    /**
     * Constructs a new PID controller.
     *
     * A PID controller is used to calculate corrections based on error values over
     * time as the difference between a desired value and the actual measured value.
     * This PID controller also limits integral windup using a max_integral value.
     *
     * Resources:
     * - https://raw.org/book/control-theory/introduction-to-pid-controllers/
     * - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-reset-windup/
     *
     * @pre max_integral must be >= 0.0
     * @throws std::invalid_argument if max_integral < 0.0
     *
     * @param k_p The proportional gain.
     * @param k_i The integral gain.
     * @param k_d The derivative gain.
     * @param max_integral The maximum absolute value that the integrator term can
     * accumulate to.
     **/
    PidController(double k_p, double k_i, double k_d, double max_integral);

    /**
     * Given an error, returns the control effort to minimize it.
     *
     * @param error The amount of error between desired and actual output
     * @param delta_time The time passed since last step, for calculating the integrator
     * and derivative.
     **/
    double step(double error, double delta_time);

    /**
     * Resets the integrator and clears the last error used for derivative calculation.
     **/
    void reset();

    double k_p;
    double k_i;
    double k_d;
    double max_integral;

   private:
    double integral_                  = 0.0;
    std::optional<double> last_error_ = std::nullopt;
};
