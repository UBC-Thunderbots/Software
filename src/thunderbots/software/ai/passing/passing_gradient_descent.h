#pragma once

#include <util/parameter/dynamic_parameters.h>
#include "ai/world/world.h"
#include "util/timestamp.h"
#include "util/gradient_descent.h"

namespace AI::Passing {

    // TODO: functions and whatnot for this?
    // TODO: this should be in it's own file and have a cpp
    class Pass {
    public:

        // The number of parameters that make up a pass
        static const size_t NUM_PARAMS = 4;

        // TODO: properly implement
        Pass() {}

        // TODO: these should probably just be `GradientDescentPass`
        // TODO: javadoc comment
        Pass(std::array<double, NUM_PARAMS> params) :
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

        // TODO: Test that outputing params from here and reading them back into the constructor results in the exact same pass
        /**
         * Get the parameters that define this Pass.
         *
         * These are the values that we actually optimize with gradient descent
         *
         * @return A vector of params defining this Pass
         */
        std::array<double, NUM_PARAMS> getParams() {
            return {receiver_point.x(), receiver_point.y(), pass_speed_m_per_s,
                    pass_start_time.getMilliseconds()};
        }
    };

    // TODO: detailed Javadoc comment for this class
    class PassGenerator {

    public:
        /**
         * Create a PassingGradientDescent
         */
        PassGenerator();

        /**
         * Updates the world
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

        // TODO: better comment here?
        /**
         * Runs a full iteration of pass optimization, pruning, and generation
         */
        void iterate();

        /**
         * Gets the best pass we know of so far
         *
         * This only returns what we know so far. For example, if called directly after
         * the world state is updated, it is unlikely to return good results (if any).
         * Gradient descent must be allowed to run for some number of iterations before
         * this can be used to get a reasonable value.
         *
         * @return The best currently known pass, or `std::nullopt` if there is no
         *         reasonable pass
         */
        std::optional<Pass> getBestPass();

    private:

        /**
         * Calculate the quality of a given pass
         *
         * @param pass The pass to rate
         * @return A value in [0,1] representing the quality of the pass
         */
        double ratePass(Pass pass);

        /**
         * Calculate the quality of a given pass, expressed in terms of params
         *
         * @param pass The pass to rate
         * @return A value in [0,1] representing the quality of the pass
         */
        double ratePass(std::array<double, Pass::NUM_PARAMS> pass);

        /**
         * Compares the quality of the two given passes
         *
         * @param pass1
         * @param pass2
         * @return pass1.quality < pass2.quality
         */
        bool comparePassQuality(Pass pass1, Pass pass2);

        /**
         * Generate a given number of passes
         *
         * This function is used to generate the initial passes that are then optimized via
         * gradient descent.
         *
         * @param num_passes_to_gen  The number of passes to generate
         *
         * @return A vector containing the requested number of passes
         */
        std::vector<Pass> generatePasses(unsigned long num_passes_to_gen);

        // This constant is used to prevent division by 0 in our implementation of Adam
        // (gradient descent)
        static constexpr double eps = 1e-8;

        // The number of passes to use for gradient descent. This is the number of passes
        // that we will be trying to optimize at any given time
        unsigned int num_gradient_descent_passes;

        // The minimum pass quality that we would consider a "reasonable" pass
        double min_reasonable_pass_quality;

        // The most recent world we know about
        World world;

        // The point we are passing from
        Point passer_point;

        // All the passes that we are currently trying to optimize in gradient descent
        std::vector<Pass> passes_to_optimize;

        // The optimizer we're using to find passes
        Util::GradientDescentOptimizer<Pass::NUM_PARAMS> optimizer;

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

