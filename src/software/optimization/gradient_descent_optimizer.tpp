/**
 * NOTE: We do not use `using namespace ...` here, because this is still a header file,
 *       and as such anything that includes `gradient_descent.h` (which includes this
 *       file), would get any namespaces we use here
 * NOTE: We do not use `ParamArray` in any of the function signatures here, as
 *       `ParamArray` is dependent on a template parameter (`NUM_PARAMS`), and so you
 *       need quite a complex expression in order to use it here. As such, it was
 *       decided that just using `std::array<...>` was the better option
 */

#pragma once

#include <cmath>

#include "software/optimization/gradient_descent_optimizer.h"

template <size_t NUM_PARAMS>
GradientDescentOptimizer<NUM_PARAMS>::GradientDescentOptimizer(
    std::array<double, NUM_PARAMS> param_weights, double gradient_approx_step_size,
    double past_gradient_decay_rate, double past_squared_gradient_decay_rate)
    : param_weights(param_weights),
      past_gradient_decay_rate(past_gradient_decay_rate),
      past_squared_gradient_decay_rate(past_squared_gradient_decay_rate),
      gradient_approx_step_size(gradient_approx_step_size)
{
}

template <size_t NUM_PARAMS>
std::array<double, NUM_PARAMS> GradientDescentOptimizer<NUM_PARAMS>::maximize(
    std::function<double(std::array<double, NUM_PARAMS>)> objective_function,
    std::array<double, NUM_PARAMS> initial_value, unsigned int num_iters)
{
    return followGradient(
        objective_function, initial_value, num_iters,
        [](double curr_value, double step) { return curr_value + step; });
}

template <size_t NUM_PARAMS>
std::array<double, NUM_PARAMS> GradientDescentOptimizer<NUM_PARAMS>::minimize(
    std::function<double(std::array<double, NUM_PARAMS>)> objective_function,
    std::array<double, NUM_PARAMS> initial_value, unsigned int num_iters)
{
    return followGradient(
        objective_function, initial_value, num_iters,
        [](double curr_value, double step) { return curr_value - step; });
}

template <size_t NUM_PARAMS>
std::array<double, NUM_PARAMS> GradientDescentOptimizer<NUM_PARAMS>::followGradient(
    std::function<double(std::array<double, NUM_PARAMS>)> objective_function,
    std::array<double, NUM_PARAMS> initial_value, unsigned int num_iters,
    std::function<double(double, double)> gradient_movement_func)
{
    // Implementation of the "Adam" algorithm. See Javadoc class comment for this
    // class (in the header) for details

    // This is basically just to change the name so the below code reads more nicely
    ParamArray params = initial_value;

    // The past gradient and squared gradient averages for each parameter
    ParamArray past_gradient_averages         = {0};
    ParamArray past_squared_gradient_averages = {0};

    for (unsigned iter = 0; iter < num_iters; iter++)
    {
        ParamArray gradient = approximateGradient(params, objective_function);

        // Get the squared gradient
        ParamArray squared_gradient = {0};
        for (unsigned int i = 0; i < NUM_PARAMS; i++)
        {
            squared_gradient.at(i) = std::pow(gradient.at(i), 2);
        }

        // Update past gradient and gradient squared averages
        for (unsigned int i = 0; i < NUM_PARAMS; i++)
        {
            past_gradient_averages.at(i) =
                past_gradient_decay_rate * past_gradient_averages.at(i) +
                (1 - past_gradient_decay_rate) * gradient.at(i);
            past_squared_gradient_averages.at(i) =
                past_squared_gradient_decay_rate * past_squared_gradient_averages.at(i) +
                (1 - past_squared_gradient_decay_rate) * squared_gradient.at(i);
        }

        // Create the bias corrected gradient and gradient square averages
        ParamArray bias_corrected_past_gradient_averages         = {0};
        ParamArray bias_corrected_past_squared_gradient_averages = {0};
        for (unsigned int i = 0; i < NUM_PARAMS; i++)
        {
            bias_corrected_past_gradient_averages.at(i) =
                past_gradient_averages.at(i) /
                (1 - std::pow(past_gradient_decay_rate, iter + 1));
            bias_corrected_past_squared_gradient_averages.at(i) =
                past_squared_gradient_averages.at(i) /
                (1 - std::pow(past_squared_gradient_decay_rate, iter + 1));
        }

        // Step each param in the direction of the gradient using the operator
        // given to this function
        for (unsigned int i = 0; i < NUM_PARAMS; i++)
        {
            params.at(i) = gradient_movement_func(
                params.at(i),
                param_weights.at(i) * bias_corrected_past_gradient_averages.at(i) /
                    (std::sqrt(bias_corrected_past_squared_gradient_averages.at(i)) +
                     eps));
        }
    }

    return params;
}

template <size_t NUM_PARAMS>
std::array<double, NUM_PARAMS> GradientDescentOptimizer<NUM_PARAMS>::approximateGradient(
    std::array<double, NUM_PARAMS> params,
    std::function<double(std::array<double, NUM_PARAMS>)> objective_function)
{
    ParamArray gradient        = {0};
    double curr_function_value = objective_function(params);

    for (unsigned i = 0; i < NUM_PARAMS; i++)
    {
        auto test_params = params;
        test_params.at(i) += gradient_approx_step_size * param_weights.at(i);
        double new_function_value = objective_function(test_params);
        gradient.at(i) =
            (new_function_value - curr_function_value) / gradient_approx_step_size;
    }

    return gradient;
}
