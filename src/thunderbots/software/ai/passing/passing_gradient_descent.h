#pragma once

#include <util/parameter/dynamic_parameters.h>
#include "ai/world/world.h"
#include "util/timestamp.h"

namespace AI::Passing {

// The number of variables we optimize
// TODO: where should this go?
static const int NUM_OPTIMIZE_PARAMS = 4;

// TODO: functions and whatnot for this?
// TODO: this should be in it's own file and have a cpp
class Pass {
public:

    // TODO: properly implement
    Pass() {}

    // TODO: these should probably just be `GradientDescentPass`
    // TODO: javadoc comment
    Pass(std::array<double, NUM_OPTIMIZE_PARAMS> params) :
            Pass(params[0], params[1], params[2], params[3]) {}

    // TODO: javadoc comment
    // TODO: decide on what to do about clamping time and velocity here.......... we clamp to stop the gradient descent from trying values outside of acceptable ranges
    Pass(double receiver_point_x, double receiver_point_y, double pass_speed_m_per_s, double pass_start_time_s) :
            Pass(Point(receiver_point_x, receiver_point_y),
                    std::max(0.0, pass_speed_m_per_s),
                    Timestamp::fromSeconds(std::max(0.0, pass_start_time_s))) {}

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
class GradientDescent {

public:
    /**
     * Create a PassingGradientDescent
     */
    GradientDescent();

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
        // TODO: properly implement this
        GradientDescentPass() {};

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

    /**
     * Optimize the given pass for the given number of iterations using gradient descent
     *
     * This runs one iteration of Adam (basically Stochastic Gradient Descent)
     *
     * @param pass The pass to optimize
     * @param max_num_iterations The number of iterations to try to optimize the path for
     * @return The pass optimized by one iteration
     */
    GradientDescent::GradientDescentPass optimizePass(GradientDescentPass pass);

    /**
     * Approximate the gradient for a given pass
     * @param pass The pass to approximate the gradient around
     * @return
     */
    std::array<double, NUM_OPTIMIZE_PARAMS>
    approximateGradient(GradientDescentPass pass);

    /**
     * Calculate the quality of a given pass
     * @param pass The pass to rate
     * @return A value in [0,1] representing the quality of the pass
     */
    double ratePass(GradientDescentPass pass);

    /**
     * Compares the quality of the two given passes like: pass1 < pass2
     *
     * @param pass1
     * @param pass2
     * @return pass1.quality < pass2.quality
     */
    bool comparePassQuality(GradientDescentPass pass1, GradientDescentPass pass2);

    /**
     * Generate a given number of paths
     *
     * This function is used to generate the initial paths that are then optimized via
     * gradient descent.
     *
     * @param num_paths_to_gen  The number of paths to generate
     *
     * @return A vector containing the requested number of paths
     */
    std::vector<GradientDescentPass> generatePaths(unsigned long num_paths_to_gen);

    // This constant is used to prevent division by 0 in our implementation of Adam
    // (gradient descent)
    static constexpr double eps = 1e-8;

    // The number of paths to use for gradient descent. This is the number of paths
    // that we will be trying to optimize at any given time
    unsigned int num_gradient_descent_paths;

    // The step size to take when approximating the gradient
    double gradient_approx_step_size;

    // The minimum pass quality that we would consider a "reasonable" path
    // TODO: set this in the constructor
    double min_reasonable_pass_quality;

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

    // Decay rates used for Adam
    // ( https://en.wikipedia.org/wiki/Stochastic_gradient_descent#Adam )
    // ( http://ruder.io/optimizing-gradient-descent/index.html#adam )
    // ( https://en.wikipedia.org/wiki/Moment_(mathematics) )
    double past_gradient_decay_rate;
    double past_squared_gradient_decay_rate;
};

// TODO: implement this in it's own file, and probably put the declaration there as well
/**
 * Calculates the static position quality for a given position on a given field
 *
 * @param field The field on which to calculate the static position quality
 * @param position The position on the field at which to calculate the quality
 *
 * @return A value in [0,1] representing the quality of the given point on the given field
 */
inline double getStaticPositionQuality(Field field, Point position) {

    double positionQuality = 1;
    double length          = field.length() / 2;
    double width           = field.width() / 2;

    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sigmoid_steepness = 15;

    // The offset from the sides of the field for the center of the sigmoid functions
    double x_offset = Util::DynamicParameters::Passing::static_position_quality_x_offset.value();
    double y_offset = Util::DynamicParameters::Passing::static_position_quality_y_offset.value();
    double goal_weight = Util::DynamicParameters::Passing::static_position_quality_friendly_goal_distance_weight.value();

    if (position.x() >= 0)
    {
        // Positive x is closer to the enemy goal, so the higher the better!
        positionQuality =
                positionQuality / (1 + std::exp(sigmoid_steepness * (position.x() - (length - x_offset))));
    }
    else if (position.x() < 0)
    {
        // Negative x is closer to our goal, so the lower the worse it is
        positionQuality =
                positionQuality / (1 + std::exp(sigmoid_steepness * (-position.x() - (length - x_offset))));
    }

    // Give a better score to positions in the center
    if (position.y() >= 0)
    {
        positionQuality =
                positionQuality / (1 + std::exp(sigmoid_steepness * (position.y() - (width - y_offset))));
    }
    else if (position.y() < 0)
    {
        positionQuality =
                positionQuality / (1 + std::exp(sigmoid_steepness * (-position.y() - (width - y_offset))));
    }

    // Add a negative weight for positions closer to our goal
    Vector vec_to_goal = Vector(field.friendlyGoal().x() - position.x(), field.friendlyGoal().y() - position.y());
    positionQuality =
            positionQuality * (1 - std::exp(goal_weight * (std::pow(2, vec_to_goal.len()))));

    return positionQuality;
}

}

