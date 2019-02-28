/**
 * Implementation of evaluation functions for passing
 */

#include <numeric>

#include "../shared/constants.h"
#include "ai/passing/evaluation.h"
#include "util/parameter/dynamic_parameters.h"

using namespace AI::Passing;

double AI::Passing::getFriendlyCapability(Team friendly_team, AI::Passing::Pass pass) {
    // We need at least one robot to pass to
    if (friendly_team.getAllRobots().empty()){
        return 0;
    }

    // Get the robot that is closest to where the pass would be received
    Robot best_receiver = friendly_team.getAllRobots()[0];
    for (Robot& robot : friendly_team.getAllRobots()){
        double distance = (robot.position() - pass.receiverPoint()).len();
        double curr_best_distance = (best_receiver.position() - pass.receiverPoint()).len();
        if (distance < curr_best_distance){
            best_receiver = robot;
        }
    }

    // Figure out what time the robot would have to receive the ball at
    Timestamp ball_travel_time = Timestamp::fromSeconds((pass.receiverPoint() - pass.passerPoint()).len() / pass.speed());
    Timestamp receive_time = pass.startTime() + ball_travel_time;

    // Figure out how long it would take our robot to get there
    Timestamp min_robot_travel_time = getTimeToPositionForRobot(best_receiver, pass.receiverPoint());
    Timestamp earliest_time_to_receive_point = best_receiver.lastUpdateTimestamp() + min_robot_travel_time;

    // TODO: Take into account rotation time

    // Create a sigmoid that goes to 0 as the time required to get to the reception
    // point exceeds the time we would need to get there by
    return 1 - sigmoid(receive_time.getSeconds(), earliest_time_to_receive_point.getSeconds() - 0.5, 1);
}

Timestamp AI::Passing::getTimeToPositionForRobot(Robot robot, Point dest) {
    // TODO:
    // (1) Create a vector that is the sum of the compoenent of the velocity perpendicular to the goal and the velocity componenet moving you *directly* away from the goal
    // (2) Figure out the time required to kill off the velocity vector from (1) and figure out where we would end up once we've killed it off (in position and velocity)
    // (3) Figure out the time required to (accounting for the robots current velocity) move to the goal
    // (3.5) Compare current stopping distance to distance to goal. If greater, "add" another sub-stage that adds the time required to fully stop to the total time
    // (3.51) Figure out the time required to get to the goal from the current position (accounting for 3.5 if needed) based on a linear acceleration profile.
    // We assume a linear acceleration profile:
    // (1) velocity = MAX_ACCELERATION*time
    // we integrate (1) to get:
    // (2) displacement = MAX_ACCELERATION/2 * time^2
    // we rearrange to get:
    // (3) time = sqrt(2 * displacement / MAX_ACCELERATION)
    // we sub. (3) into (1) to get:
    // (4) velocity = MAX_ACCELERATION*sqrt(2 * displacement / MAX_ACCELERATION)
    // and rearrange to get:
    // (5) displacement = (velocity / MAX_ACCELERATION)^2 * MAX_ACCELERATION/2
    // We re-arrange (3) to get:
    // (6) displacement = time^2 * MAX_ACCELERATION/2

    // We approximate the time required to get to goal by summing the time required to
    // stop moving, and the time required to go from wherever we managed to stop moving
    // by to the goal

    double max_accel = ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
    double max_speed = ROBOT_MAX_SPEED_METERS_PER_SECOND;

    // Calculate the time required to stop the robot moving
    double initial_speed = robot.velocity().len();
    double time_to_stop = max_accel / initial_speed;

    // Calculate how far the robot moved while it was trying to stop
    // (x = ut + 1/2 * at^2)
    double stop_distance = initial_speed*time_to_stop + 1.0/2.0 * max_accel*std::pow(time_to_stop, 2);
    Point robot_stop_position = robot.position() + robot.velocity().norm() * stop_distance;

    double dist = (robot_stop_position - dest).len();
    // Calculate the distance required to reach max possible velocity of the robot
    // using (5)
    double dist_to_max_possible_vel = std::pow(max_speed, 2) * max_accel/2;

    // Calculate how long we'll accelerate for using (6)
    double acceleration_time = std::sqrt(2 * std::min(dist/2, dist_to_max_possible_vel) / max_accel);

    // Calculate how long we'll be at the max possible velocity (if any time at all)
    double time_at_max_vel = std::max(0.0, dist - 2*dist_to_max_possible_vel) / max_speed;

    // The time taken to get to the receiver point is:
    // time to accelerate + time at the max velocity + time to de-accelerate
    // Note that the acceleration time is the same as a de-acceleration time
    double travel_time = 2*acceleration_time + time_at_max_vel;

    return Timestamp::fromSeconds(travel_time);
}

double AI::Passing::getStaticPositionQuality(const Field& field, const Point& position)
{
    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sig_width = 0.1;

    // The offset from the sides of the field for the center of the sigmoid functions
    double x_offset =
        Util::DynamicParameters::AI::Passing::static_field_position_quality_x_offset
            .value();
    double y_offset =
        Util::DynamicParameters::AI::Passing::static_field_position_quality_y_offset
            .value();
    double friendly_goal_weight =
        Util::DynamicParameters::AI::Passing::
            static_field_position_quality_friendly_goal_distance_weight.value();

    // Make a slightly smaller field, and positive weight values in this reduced field
    double half_field_length = field.length() / 2;
    double half_field_width  = field.width() / 2;
    Rectangle reduced_size_field(
        Point(-half_field_length + x_offset, -half_field_width + y_offset),
        Point(half_field_length - x_offset, half_field_width - y_offset));
    double on_field_quality = rectangleSigmoid(reduced_size_field, position, sig_width);

    // Add a negative weight for positions closer to our goal
    Vector vec_to_friendly_goal      = Vector(field.friendlyGoal().x() - position.x(),
                                         field.friendlyGoal().y() - position.y());
    double distance_to_friendly_goal = vec_to_friendly_goal.len();
    double near_friendly_goal_quality =
        (1 -
         std::exp(-friendly_goal_weight * (std::pow(5, -2 + distance_to_friendly_goal))));

    // Add a strong negative weight for positions within the enemy defense area, as we
    // cannot pass there
    double in_enemy_defense_area_quality =
        1 - rectangleSigmoid(field.enemyDefenseArea(), position, sig_width);

    return on_field_quality * near_friendly_goal_quality * in_enemy_defense_area_quality;
}

double AI::Passing::rectangleSigmoid(const Rectangle& rect, const Point& point,
                                     const double& sig_width)
{
    double x_offset = rect.centre().x();
    double y_offset = rect.centre().y();
    double x_size   = rect.width() / 2;
    double y_size   = rect.height() / 2;
    double x        = point.x();
    double y        = point.y();

    // For both x and y here we use two sigmoid functions centered at the positive and
    // negative edge of the rectangle respectively

    double x_val = std::min(sigmoid(x, x_offset + x_size, -sig_width),
                            sigmoid(x, x_offset - x_size, sig_width));

    double y_val = std::min(sigmoid(y, y_offset + y_size, -sig_width),
                            sigmoid(y, y_offset - y_size, sig_width));

    return x_val * y_val;
}

double AI::Passing::circleSigmoid(const Circle& circle, const Point& point,
                                  const double& sig_width)
{
    // Calculate how far the point is from the circle center
    double distance_from_circle_center = (point - circle.getOrigin()).len();

    return sigmoid(distance_from_circle_center, circle.getRadius(), -sig_width);
}

double AI::Passing::sigmoid(const double& v, const double& offset,
                            const double& sig_width)
{
    // This is factor that changes how quickly the sigmoid goes from 0 to 1 it. We divide
    // 8 by it because that is the distance a sigmoid function centered about 0 takes to
    // go from 0.018 to 0.982 (and that is what the `sig_width` is, as per the javadoc
    // comment for this function)
    double sig_change_factor = 8 / sig_width;

    return 1 / (1 + std::exp(sig_change_factor * (offset - v)));
}
