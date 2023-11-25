#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"

#include "software/geom/algorithms/closest_point.h"

bool PassDefenderFSM::passStarted(const Update& event)
{
    auto ball_position = event.common.world.ball().position();
    Vector ball_receiver_point_vector(
        event.control_params.position_to_block_from.x() - ball_position.x(),
        event.control_params.position_to_block_from.y() - ball_position.y());

    bool pass_started = event.common.world.ball().hasBallBeenKicked(
        ball_receiver_point_vector.orientation(), MIN_PASS_SPEED,
        MAX_PASS_ANGLE_DIFFERENCE);

    if (pass_started)
    {
        // We want to keep track of the initial trajectory of the pass
        // so that we can later tell whether the ball strays from
        // this trajectory
        pass_orientation = event.common.world.ball().velocity().orientation();
    }

    return pass_started;
}

bool PassDefenderFSM::ballDeflected(const Update& event)
{
    auto orientation_difference =
        event.common.world.ball().velocity().orientation().minDiff(pass_orientation);

    // If the ball strays from the initial trajectory of the pass,
    // it was likely deflected off course by another robot or chipped
    // away by the pass defender
    return orientation_difference.abs() > MIN_DEFLECTION_ANGLE;
}

void PassDefenderFSM::blockPass(const Update& event)
{
    auto position_to_block_from = event.control_params.position_to_block_from;
    auto ball_position          = event.common.world.ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    // Face the ball and move to position_to_block_from, which should be a location
    // on the field that blocks a passing lane between two enemy robots
    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(position_to_block_from), face_ball_orientation, 0, false,
        TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants()));
}

void PassDefenderFSM::interceptBall(const Update& event)
{
    auto ball           = event.common.world.ball();
    auto robot_position = event.common.robot.position();

    if ((ball.position() - robot_position).length() >
        BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING)
    {
        Point intercept_position = ball.position();
        if (ball.velocity().length() != 0)
        {
            // Find the closest point on the line of the ball's current trajectory
            // that the defender can move to and intercept the pass
            intercept_position = closestPoint(
                robot_position, Line(ball.position(), ball.position() + ball.velocity()));
        }

        auto face_ball_orientation = (ball.position() - robot_position).orientation();

        // Move to intercept the pass by positioning defender in front of the
        // ball's current trajectory
        event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(intercept_position), face_ball_orientation, 0, false,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
            event.common.robot.robotConstants(), std::optional<double>()));
    }
}
