/**
 * Implementation of evaluation functions for passing
 */

// TODO: declare things const& and whatnot to maybe improve speed

#include "ai/passing/evaluation.h"
#include "util/parameter/dynamic_parameters.h"

using namespace AI::Passing;

double AI::Passing::getStaticPositionQuality(Field field, Point position) {
    double length          = field.length() / 2;
    double width           = field.width() / 2;

    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sigmoid_width = 0.1;

    // The offset from the sides of the field for the center of the sigmoid functions
    // TODO: should we be using dynamic parameters like this here?
    double x_offset = Util::DynamicParameters::AI::Passing::static_position_quality_x_offset.value();
    double y_offset = Util::DynamicParameters::AI::Passing::static_position_quality_y_offset.value();
    double goal_weight = Util::DynamicParameters::AI::Passing::static_position_quality_friendly_goal_distance_weight.value();

    // Make a slightly smaller field, and positive weight values in this reduced field
    Rectangle reduced_size_field(
            Point(-length + x_offset, -width + y_offset),
            Point( length - x_offset, width - y_offset)
    );
    double on_field_quality = rectangleSigmoid(reduced_size_field, position, sigmoid_width);

    // Add a negative weight for positions closer to our goal
    Vector vec_to_goal = Vector(field.friendlyGoal().x() - position.x(), field.friendlyGoal().y() - position.y());
    double distance_to_goal = vec_to_goal.len();
    double near_goal_quality =
             (1 - std::exp(-goal_weight * (std::pow(5, -2+distance_to_goal))));

    return on_field_quality * near_goal_quality;
}

double AI::Passing::rectangleSigmoid(Rectangle rect, Point point, double sig_width){
    double x_offset = rect.centre().x();
    double y_offset = rect.centre().x();
    double x_size = rect.width()/2;
    double y_size = rect.height()/2;
    double x = point.x();
    double y = point.y();

    double x_val = std::min(
            sigmoid(x, x_offset + x_size, -sig_width),
            sigmoid(x, x_offset - x_size, sig_width)
            );

    double y_val = std::min(
            sigmoid(y, y_offset + y_size, -sig_width),
            sigmoid(y, y_offset - y_size, sig_width)
    );

    return x_val * y_val;
}

double AI::Passing::sigmoid(double v, double offset, double sig_width) {
    // This is factor that changes how quickly the sigmoid goes from 0 outside the
    // rectangle to 1 inside it. We multiply it by 4 because that is the distance a
    // sigmoid function centered about 0 takes to go from 0 to 0.982
    double sig_change_factor =  4/sig_width;

    return 1/(1+std::exp(sig_change_factor*(offset-v)));
}
