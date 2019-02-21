/**
 * Implementation of evaluation functions for passing
 */

#include "ai/passing/evaluation.h"
#include "util/parameter/dynamic_parameters.h"

using namespace AI:Passing;

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

double rectangleSigmoid(Rectangle rect, Point point, double sig_width){
    double x_offset = rect.centre().x();
    double y_offset = rect.centre().x();
    double x_size = rect.width()/2;
    double y_size = rect.height()/2;
    double x = point.x();
    double y = point.y();
    // This is factor that changes how quickly the sigmoid goes from 0 outside the
    // rectangle to 1 inside it
    double sig_change_factor = sig_width * 4;

    double negative_x_sigmoid_value = 1/(1+std::exp(sig_change_factor*(-x_size-x)));
    double positive_x_sigmoid_value = 1/(1+std::exp(sig_change_factor*(-x_size+x)));
    double x_val = std::min(negative_x_sigmoid_value, positive_x_sigmoid_value);

    double negative_y_sigmoid_value = 1/(1+std::exp(sig_change_factor*(-y_size-y)));
    double positive_y_sigmoid_value = 1/(1+std::exp(sig_change_factor*(-y_size+y)));
    double y_val = std::min(negative_y_sigmoid_value, positive_y_sigmoid_value);

    return x_val * y_val;
}
