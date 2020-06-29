/**
 * This file contains the declaration for the GradientDescentOptimizer
 */
#pragma once

#include <algorithm>
#include <array>
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
 * http://ruder.io/optimizing-gradient-descent/index.html#adam
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
    // http://ruder.io/optimizing-gradient-descent/index.html#adam
    static constexpr double DEFAULT_PAST_GRADIENT_DECAY_RATE         = 0.9;
    static constexpr double DEFAULT_PAST_SQUARED_GRADIENT_DECAY_RATE = 0.999;

    // Default step size for approximating the gradient of functions
    static constexpr double DEFAULT_GRADIENT_APPROX_STEP_SIZE = 0.00001;

    /**
     * Creates a GradientDescentOptimizer
     *
     * This constructor chooses usually sane values for both
     * "past_gradient_decay_rate" and "past_squared_gradient_decay_rate", and uses a
     * value of 1 for each param weight
     */
    GradientDescentOptimizer();

    /**
     * Creates a GradientDescentOptimizer
     *
     * This constructor chooses usually sane values for both
     * "past_gradient_decay_rate" and "past_squared_gradient_decay_rate"
     *
     * @param param_weights The weights to multiply each parameter by when calculating
     *                      the gradient of the objective function.
     */
    explicit GradientDescentOptimizer(ParamArray param_weights);

    /**
     * Creates a GradientDescentOptimizer
     *
     * This constructor chooses usually sane values for both
     * "past_gradient_decay_rate" and "past_squared_gradient_decay_rate"
     *
     * @param param_weights The weight to multiply
     * @param gradient_approx_step_size The size of step to take forward when
     *                                  approximating the gradient of a function
     */
    GradientDescentOptimizer(ParamArray param_weights, double gradient_approx_step_size);

    /**
     * Creates a GradientDescentOptimizer
     *
     * NOTE: Unless you know what you're doing, you probably don't want to use this
     * constructor. Instead, use one of the other constructors that specifies the
     * decay rates for you, as they use values that are almost always good.
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
    GradientDescentOptimizer(ParamArray param_weights, double gradient_approx_step_size,
                             double past_gradient_decay_rate,
                             double past_squared_gradient_decay_rate);

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

#include "software/optimization/gradient_descent_optimizer.tpp"
