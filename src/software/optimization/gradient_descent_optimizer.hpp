#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

/**
 * This class implements a version of Stochastic Gradient Descent (SGD), namely Adam
 * (see links below for details). It provides functionality for both maximizing
 * and minimizing arbitrary functions. For example usage, please see the tests.
 *
 * As this class is templated, it is header-only. To split up definition and
 * implementation of functions has been moved to a `.tpp` file that is included at
 * the end of this file.
 *
 * "Weights" are used throughout this class, and hence are documented here as follows:
 *      Weights used to normalize parameters, as gradient descent works much better if
 *      the function being optimized is homogeneous in direction
 *      for example, f = x^2 + y^2 is easier to optimize than f = x^2 + 50*y^2
 *
 * This class uses an implementation of "Adam" (Adaptive Moment) Gradient Descent:
 * https://machinelearningmastery.com/adam-optimization-algorithm-for-deep-learning/
 * https://en.wikipedia.org/wiki/Stochastic_gradient_descent#Adam
 * https://www.ruder.io/optimizing-gradient-descent/#adam
 * https://en.wikipedia.org/wiki/Moment_(mathematics)
 *
 * NOTE: CLion complains about "Redefinition of GradientDescentOptimizer", but it's
 *       incorrect, this class compiles just fine.
 *
 * @tparam NUM_PARAMS The number of parameters that a given instance of this class
 *                    will optimize over.
 */
template <size_t NUM_PARAMS>
class GradientDescentOptimizer
{
   public:
    using ParamArray = std::array<double, NUM_PARAMS>;

    // Almost always good values for the decay rates, taken from:
    // https://www.ruder.io/optimizing-gradient-descent/#adam
    static constexpr double DEFAULT_PAST_GRADIENT_DECAY_RATE         = 0.9;
    static constexpr double DEFAULT_PAST_SQUARED_GRADIENT_DECAY_RATE = 0.999;

    // Default step size for approximating the gradient of functions
    static constexpr double DEFAULT_GRADIENT_APPROX_STEP_SIZE = 0.00001;

    /**
     * Creates a GradientDescentOptimizer
     *
     * NOTE: Unless you know what you're doing, you probably don't want specify the
     * decay rates, as the default values are almost always good.
     *
     * @param param_weights The weight to multiply
     * @param gradient_approx_step_size The size of step to take forward when
     *                                  approximating the gradient of a function
     * @param past_gradient_decay_rate Past gradient knowledge decay rate, see
     *                                 corresponding class member variable for details
     * @param past_squared_gradient_decay_rate Past squared gradient knowledge decay
     *                                         rate, see corresponding class member
     *                                         variable for details
     */
    GradientDescentOptimizer(
        ParamArray param_weights = GradientDescentOptimizer<NUM_PARAMS>::ParamArray{1},
        double gradient_approx_step_size = DEFAULT_GRADIENT_APPROX_STEP_SIZE,
        double past_gradient_decay_rate  = DEFAULT_PAST_GRADIENT_DECAY_RATE,
        double past_squared_gradient_decay_rate =
            DEFAULT_PAST_SQUARED_GRADIENT_DECAY_RATE);

    /**
     * Attempts to maximize the given objective function
     *
     * Runs gradient descent, starting from the given initial_value and running for
     * num_iters
     *
     * @param objective_function The function to maximize
     * @param initial_value The value to start from
     * @param num_iters The number of iterations to run for
     *
     * @return The parameters corresponding to the maximum value of the objective
     *         found
     */
    ParamArray maximize(std::function<double(ParamArray)> objective_function,
                        ParamArray initial_value, unsigned int num_iters);

    /**
     * Attempts to minimize the given objective function
     *
     * Runs gradient descent, starting from the given initial_value and running for
     * num_iters
     *
     * @param objective_function The function to minimize
     * @param initial_value The value to start from
     * @param num_iters The number of iterations to run for
     *
     * @return The parameters corresponding to the minimum value of the objective
     *         found
     */
    ParamArray minimize(std::function<double(ParamArray)> objective_function,
                        ParamArray initial_value, unsigned int num_iters);


   private:
    /**
     * Attempts to minimize or maximize the given objective function
     *
     * Runs gradient descent, starting from the given initial_value and running for
     * num_iters
     *
     * @param objective_function The function to minimize
     * @param initial_value The value to start from
     * @param num_iters The number of iterations to run for
     * @param gradient_movement_func The function to use on each step along the
     *                               gradient, either "-" to minimize the given
     *                               function, or "+" to maximize it
     *
     * @return The parameters corresponding to the minimum or maximum value of the
     *         objective found, depending on what gradient_movement_func was given
     */
    ParamArray followGradient(
        std::function<double(ParamArray)> objective_function, ParamArray initial_value,
        unsigned int num_iters,
        std::function<double(double, double)> gradient_movement_func);

    /**
     * Approximate the gradient of the objective function around a given point
     *
     * @param params The params around which we want to approximate the gradient
     * @param objective_function The function to approximate the gradient over
     * @return A ParamArray, where each "param" is the derivative with respect to the
     *         corresponding input param.
     */
    ParamArray approximateGradient(ParamArray params,
                                   std::function<double(ParamArray)> objective_function);

    // This constant is used to prevent division by 0 in our implementation of Adam
    // (gradient descent)
    static constexpr double eps = 1e-8;

    // Weights used to normalize parameters. See class javadoc comment above for
    // details
    ParamArray param_weights;

    // Decay rates used for Adam (see class description for details)
    double past_gradient_decay_rate;
    double past_squared_gradient_decay_rate;

    // The size of step of take when numerically approximating the derivative of
    // an objective function
    double gradient_approx_step_size;
};

/**
 * NOTE: We do not use `using namespace ...` here, because this is still a header file,
 *       and as such anything that includes `gradient_descent.h` (which includes this
 *       file), would get any namespaces we use here
 * NOTE: We do not use `ParamArray` in any of the function signatures here, as
 *       `ParamArray` is dependent on a template parameter (`NUM_PARAMS`), and so you
 *       need quite a complex expression in order to use it here. As such, it was
 *       decided that just using `std::array<...>` was the better option
 */

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
                (1 - std::pow(past_gradient_decay_rate, 2));
            bias_corrected_past_squared_gradient_averages.at(i) =
                past_squared_gradient_averages.at(i) /
                (1 - std::pow(past_squared_gradient_decay_rate, 2));
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
