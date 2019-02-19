#pragma once

#include <array>
#include <algorithm>
#include <functional>

// YOU ARE HERE!!!! Need to finish making the `GradientDescentOptimizer` class and open a
// PR for it!!

namespace Util {

    // TODO: detailed Javadoc comment for this class
    // TODO: better class name??  Ask mathhew for suggestions
    template <unsigned int NUM_PARAMS>
    class GradientDescentOptimizer {

    public:
        // Delete the default constructor
        GradientDescentOptimizer() = delete;

        /**
         * Creates a GradientDescentOptimizer
         *
         * @param objective_function The function that is minimized for a given set of
         * params
         * @param gradient_approx_step_size The size of step to take forward when
         *                                  approximating the gradient
         * @param param_weights The weight to multiply
         * @param past_gradient_decay_rate
         * @param past_square_gradient_decay_rate
         */
        GradientDescentOptimizer(
                std::function<double(std::array<double, NUM_PARAMS>)> objective_function,
                double gradient_approx_step_size,
                std::array<double, NUM_PARAMS> param_weights,
                double past_gradient_decay_rate,
                double past_square_gradient_decay_rate
                );

        // TODO: make this comment better?
        /**
         * Run gradient descent
         *
         * Runs gradient descent for the given number of iterations over the given param
         *
         * @param params the parameters to optimize
         * @param num_iterations the number of iterations to run gradient descent for
         *
         * @return The given params optimized for num_iterations
         */
        std::array<double, NUM_PARAMS> runGradientDescent(
                std::array<double, NUM_PARAMS> params, unsigned int num_iterations);

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
        GradientDescentOptimizer::GradientDescentPass optimizePass(GradientDescentPass pass);

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


}
