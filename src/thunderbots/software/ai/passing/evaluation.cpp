// TODO: start of file comment

#include "ai/passing/evaluation.h"
#include "util/parameter/dynamic_parameters.h"

double getStaticPositionQuality(Field field, Point position) {

    double positionQuality = 1;
    double length          = field.length() / 2;
    double width           = field.width() / 2;

    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sigmoid_steepness = 15;

    // The offset from the sides of the field for the center of the sigmoid functions
    // TODO: should we be using dynamic parameters like this here?
    double x_offset = Util::DynamicParameters::AI::Passing::static_position_quality_x_offset.value();
    double y_offset = Util::DynamicParameters::AI::Passing::static_position_quality_y_offset.value();
    double goal_weight = Util::DynamicParameters::AI::Passing::static_position_quality_friendly_goal_distance_weight.value();

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
