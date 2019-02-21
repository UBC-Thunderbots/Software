/**
 * Implementation of evaluation functions for passing
 */

// TODO: declare things const& and whatnot to maybe improve speed

#include "ai/passing/evaluation.h"
#include "util/parameter/dynamic_parameters.h"

using namespace AI::Passing;

double AI::Passing::getStaticPositionQuality(Field field, Point position) {
    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sigmoid_width = 0.1;

    // The offset from the sides of the field for the center of the sigmoid functions
    // TODO: should we be using dynamic parameters like this here?
    double x_offset = Util::DynamicParameters::AI::Passing::static_position_quality_x_offset.value();
    double y_offset = Util::DynamicParameters::AI::Passing::static_position_quality_y_offset.value();
    double friendly_goal_weight = Util::DynamicParameters::AI::Passing::static_position_quality_friendly_goal_distance_weight.value();

    // Make a slightly smaller field, and positive weight values in this reduced field
    double half_field_length          = field.length() / 2;
    double half_field_width           = field.width() / 2;
    Rectangle reduced_size_field(
            Point(-half_field_length + x_offset, -half_field_width + y_offset),
            Point( half_field_length - x_offset, half_field_width - y_offset)
    );
    double on_field_quality = rectangleSigmoid(reduced_size_field, position, sigmoid_width);

    // Add a negative weight for positions closer to our goal
    Vector vec_to_friendly_goal = Vector(field.friendlyGoal().x() - position.x(), field.friendlyGoal().y() - position.y());
    double distance_to_friendly_goal = vec_to_friendly_goal.len();
    double near_friendly_goal_quality =
             (1 - std::exp(-friendly_goal_weight * (std::pow(5, -2+distance_to_friendly_goal))));

    // Add a strong negative weight for positions within the enemy defense area, as we
    // cannot pass there
    double in_enemy_defense_area_quality =
            1 - rectangleSigmoid(field.enemyDefenseArea(), position, 0.1);

    return on_field_quality * near_friendly_goal_quality * in_enemy_defense_area_quality;
}

double AI::Passing::rectangleSigmoid(Rectangle rect, Point point, double sig_width ){
    double x_offset = rect.centre().x();
    double y_offset = rect.centre().y();
    double x_size = rect.width()/2;
    double y_size = rect.height()/2;
    double x = point.x();
    double y = point.y();

    // For both x and y here we use two sigmoid functions centered at the positive and
    // negative edge of the rectangle respectively

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

double AI::Passing::circleSigmoid(Circle circle, Point point, double sig_width) {
    // Calculate how far the point is from the circle center
    double distance_from_circle_center = (point - circle.getOrigin()).len();

    // Here we use two sigmoids mirrored over the center of the circle, with the center
    // of each sigmoid lying on the +radius, -radius respectively
    double sig_value = std::min(
            sigmoid(distance_from_circle_center, circle.getRadius(), -sig_width),
            sigmoid(distance_from_circle_center, -circle.getRadius(), sig_width)
            );

    return sig_value;
}

double AI::Passing::sigmoid(double v, double offset, double sig_width) {
    // This is factor that changes how quickly the sigmoid goes from 0 outside the
    // rectangle to 1 inside it. We divide 8 by it because that is the distance a
    // sigmoid function centered about 0 takes to go from 0.018 to 0.982
    // (and that is what the `sig_width` is, as per the javadoc comment for this function)
    double sig_change_factor =  8/sig_width;

    return 1/(1+std::exp(sig_change_factor*(offset-v)));
}
