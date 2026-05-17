#pragma once

#include <optional>

namespace controls
{
    template<typename T>
    class PIDController
    {
        public:
        /**
        * Constructs a new PID controller.
        **/
        PIDController(T k_p, T k_i, T k_d, T max_integral);

        /**
        * Given an error, returns the control effort to minimize it.
        *
        * @param deltaTime The time passed since last step, for calculating integrator and derivative. If this function
        * is calling in invariant intervals, deltaTime can be set to 1 and any effects it would have can be handled by
        * tuning kD.
        **/
        T step(T error, T delta_time=1.0f);

        /**
        * Resets the integrator and clears the last error used for derivative calculation.
        **/
        void reset();

        T k_p;
        T k_i;
        T k_d;
        T max_integral;

        protected:
        T integral;
        std::optional<T> last_error = std::nullopt;
    };
}  // namespace controls

template<typename T>
controls::PIDController<T>::PIDController(T k_p, T k_i, T k_d, T max_integral) :
    k_p(k_p),
    k_i(k_i),
    k_d(k_d),
    max_integral(max_integral)
    {};

template<typename T>
T controls::PIDController<T>::step(T error, T delta_time)
{
    // If sign of error swaps, reset integrator
    if (last_error.value_or(0) * error < 0)
    {
        integral = 0;
    }

    integral += std::max(std::min(error * delta_time, max_integral), -max_integral);
    // set derivative, if no last_error, just set to 0
    const T derivative = (last_error.value_or(error) - error) / delta_time;
    last_error = error;
    return error * k_p + integral * k_i + derivative * k_d;
}

template<typename T>
void controls::PIDController<T>::reset()
{
    integral = 0;
    last_error = std::nullopt;
}

