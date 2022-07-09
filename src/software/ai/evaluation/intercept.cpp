#include "software/ai/evaluation/intercept.h"

#include "shared/constants.h"
#include "software/ai/evaluation/time_to_travel.h"
#include "software/geom/algorithms/contains.h"
#include "software/optimization/gradient_descent_optimizer.hpp"
#include "software/geom/algorithms/distance.h"

Point robotPositionToFaceBall(const Point &ball_position,
                                          const Angle &face_ball_angle,
                                          double additional_offset)
{
    return ball_position - Vector::createFromAngle(face_ball_angle)
            .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                       BALL_MAX_RADIUS_METERS + additional_offset);
}

std::optional<InterceptionResult> findBestInterceptForBall(const Ball &ball,
                                                           const Field &field,
                                                           const Robot &robot, bool include_fallback_interceptions)
{
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD   = 0.3;
    static constexpr double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;
    static constexpr double MAX_INTERCEPT_SPEED = 3.0;

    Point intercept_position          = ball.position();
    double interception_final_speed   = 0;
    Point fallback_interception_point = ball.position();
    double fallback_interception_final_speed =
            robot.robotConstants().robot_max_speed_m_per_s;
    Duration robot_time_to_pos = Duration();

    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        auto face_ball_vector = (ball.position() - robot.position());
        auto point_in_front_of_ball =
                robotPositionToFaceBall(ball.position(), face_ball_vector.orientation());
        robot_time_to_pos = robot.getTimeToPosition(intercept_position);

        return std::make_optional<InterceptionResult>({point_in_front_of_ball, robot_time_to_pos, 0.0});
    }

    //todo find how quick robot can be moving for it to trap the ball in dribbler when intercepting
    // if large enough then use fallback code with with that set as maximum speed.
    while (contains(field.fieldLines(), intercept_position))
    {
        std::optional<Duration> ball_time_to_position =
                ball.getTimeToMoveDistance(distance(intercept_position, ball.position()));

        // go to the stopping position of the ball
        if (!ball_time_to_position.has_value())
        {
            break;
        }

        robot_time_to_pos = robot.getTimeToPosition(intercept_position);

        if (robot_time_to_pos < ball_time_to_position.value())
        {
            break;
        }

        Vector dist_vector = intercept_position - robot.position();

        double final_speed_to_reach_in_time =
                2 * dist_vector.length() / (ball_time_to_position.value().toSeconds()) -
                robot.currentState().velocity().dot(dist_vector.normalize());
        double average_acceleration_to_reach_in_time =
                final_speed_to_reach_in_time -
                robot.currentState().velocity().dot(dist_vector.normalize()) /
                ball_time_to_position.value().toSeconds();

        if (final_speed_to_reach_in_time < MAX_INTERCEPT_SPEED && final_speed_to_reach_in_time < fallback_interception_final_speed &&
            average_acceleration_to_reach_in_time <
            robot.robotConstants().robot_max_acceleration_m_per_s_2)
        {
            fallback_interception_final_speed = final_speed_to_reach_in_time;
            fallback_interception_point       = intercept_position;
        }

        intercept_position +=
                ball.velocity().normalize(INTERCEPT_POSITION_SEARCH_INTERVAL);
    }

//     if we can't reach the ball in time and we have valid fallback interception point,
//     use it
    if (!contains(field.fieldLines(), intercept_position)){
        if (include_fallback_interceptions &&
            fallback_interception_point != ball.position())
        {
            intercept_position       = fallback_interception_point;
            interception_final_speed = fallback_interception_final_speed;
            robot_time_to_pos = robot.getTimeToPosition(fallback_interception_point, (ball.position() - robot.position()).normalize(fallback_interception_final_speed));
        } else{
            return {};
        }
    }

    return std::make_optional<InterceptionResult>({intercept_position, robot_time_to_pos, interception_final_speed});
}


