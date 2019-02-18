#pragma once

#include "ai/world/world.h"
#include "util/timestamp.h"


// The number of variables we optimize
// TODO: where should this go?
static const int NUM_OPTIMIZE_PARAMS = 4;

// TODO: functions and whatnot for this?
// TODO: this should be in it's own file and have a cpp
class Pass {
public:

    // TODO: javadoc comment
    Pass(std::array<double, NUM_OPTIMIZE_PARAMS> params) :
            Pass(params[0], params[1], params[2], params[3]) {}

    // TODO: javadoc comment
    Pass(double receiver_point_x, double receiver_point_y, double pass_speed_m_per_s, double pass_start_time_s) :
    Pass(Point(receiver_point_x, receiver_point_y), pass_speed_m_per_s, Timestamp::fromSeconds(pass_start_time_s)) {}

    // TODO: javadoc comment
    Pass(Point receiver_point, double pass_speed_m_per_s, Timestamp pass_start_time) :
    receiver_point(receiver_point), pass_speed_m_per_s(pass_speed_m_per_s), pass_start_time(pass_start_time) {}

    // The location of the receiver
    Point receiver_point;

    // The speed of the pass in meters/second
    double pass_speed_m_per_s;

    // The time to preform the pass at
    Timestamp pass_start_time;

    /**
     * Get the parameters that define this Pass.
     *
     * These are the values that we actually optimize with gradient descent
     *
     * @return A vector of params defining this Pass (TODO: full description here)
     */
    std::array<double, NUM_OPTIMIZE_PARAMS> getParams() {
        return {receiver_point.x(), receiver_point.y(), pass_speed_m_per_s,
                pass_start_time.getMilliseconds()};
    }
};

// TODO: detailed Javadoc comment for this class
class PassingGradientDescent {

public:
    // Delete the default constructor
    PassingGradientDescent() = delete;

    /**
     * Create a PassingGradientDescent
     *
     * @param num_gradient_descent_points The number of points to use for gradient descent
     * @param gradient_approx_step_size The size of the step to take when numerically
     *                                  approximating the gradient
     */
    PassingGradientDescent(unsigned int num_gradient_descent_points, double gradient_approx_step_size);

    /**
     * Updates the world used for gradient descent
     *
     * @param world
     */
    void setWorld(World world);

    /**
     * Updates the point that we are passing from
     *
     * @param passer_point the point we are passing from
     */
    void setPasserPoint(Point passer_point);

    // TODO: Ability to set a target region

    /**
     * Run gradient descent
     *
     * Runs gradient descent for the given number of iterations using the current world,
     * pruning bad points and replace them with pseudo-random ones when it is done
     *
     * @param num_iterations the number of iterations to run gradient descent for
     */
    void runGradientDescent(unsigned int num_iterations);

     /**
      * Gets the best pass we know of so far
      *
     * This only returns what we know so far. For example, if called directly after the
     * world state is updated, it is unlikely to return good results (if any). Gradient
     * descent must be allowed to run for some number of iterations before this can be
     * used to get a reasonable value.
      *
      * @return The best currently known pass, or `std::nullopt` if there is no reasonable
      *         pass
      */
    std::optional<Pass> getBestPass();

private:

    // A Pass with additional information for gradient descent
    class GradientDescentPass : public Pass {
    public:
        // TODO: javadoc comment
        GradientDescentPass(std::array<double, NUM_OPTIMIZE_PARAMS> params) : Pass(params) {};

        // TODO: make sure this comment is still accuarate
        // These values are used for the concept of "momentum" in gradient descent
        // (https://en.wikipedia.org/wiki/Stochastic_gradient_descent#Momentum)
        // We have one per parameter because we're using an implementation of Adam:
        // ( https://en.wikipedia.org/wiki/Stochastic_gradient_descent#Adam )
        // ( http://ruder.io/optimizing-gradient-descent/index.html#adam )
        // ( https://en.wikipedia.org/wiki/Moment_(mathematics) )
        std::array<double, NUM_OPTIMIZE_PARAMS> past_gradient_averages;
        std::array<double, NUM_OPTIMIZE_PARAMS> past_squared_gradient_averages;
    };

    static const double eps = std::pow(10, -8);

    /**
     * Optimize the given pass for the given number of iterations using gradient descent
     *
     * This runs one iteration of Adam (basically Stochastic Gradient Descent)
     *
     * @param pass The pass to optimize
     * @param max_num_iterations The number of iterations to try to optimize the path for
     * @return The pass optimized by one iteration
     */
    PassingGradientDescent::GradientDescentPass optimizePass(GradientDescentPass pass);

    /**
     * Approximate the gradient
     * @param pass
     * @param step_size
     * @return
     */
    std::array<double, NUM_OPTIMIZE_PARAMS> approximateGradient(GradientDescentPass pass);

    // The number of points to use for gradient descent
    unsigned int num_gradient_descent_points;

    // The step size to take when approximating the gradient
    double gradient_approx_step_size;

    // The most recent world we know about
    World world;

    // The point we are passing from
    Point passer_point;

    // All the passes that we are currently trying to optimize in gradient descent
    std::vector<GradientDescentPass> passes_to_optimize;

    // Weights used to normalize coordinates
    // gradient descent works much better if the function being optimized is
    // homogenous in direction
    // for example f = x^2 + y^2 is easier to optimize than f = x^2 + 50*y^2
    // the weights are used as an attempt to make the ratePass function more
    // homogenous
    std::array<double, NUM_OPTIMIZE_PARAMS> param_weights;

    // TODO: set these in the constructor
    // Decay rates used for Adam
    // ( https://en.wikipedia.org/wiki/Stochastic_gradient_descent#Adam )
    // ( http://ruder.io/optimizing-gradient-descent/index.html#adam )
    // ( https://en.wikipedia.org/wiki/Moment_(mathematics) )
    double past_gradient_decay_rate;
    double past_squared_gradient_decay_rate;
};
