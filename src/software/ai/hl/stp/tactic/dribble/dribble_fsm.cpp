#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"

Point DribbleFSM::robotPositionToFaceBall(const Point &ball_position,
                                          const Angle &face_ball_angle)
{
    return ball_position -
           Vector::createFromAngle(face_ball_angle)
               .normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);
}

Point DribbleFSM::findInterceptionPoint(const Robot &robot, const Ball &ball,
                                        const Field &field)
{
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD   = 0.3;
    static constexpr double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;
    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        auto face_ball_vector = (ball.position() - robot.position());
        auto point_in_front_of_ball =
            robotPositionToFaceBall(ball.position(), face_ball_vector.orientation());
        return point_in_front_of_ball;
    }
    Point intercept_position = ball.position();
    while (contains(field.fieldLines(), intercept_position))
    {
        Duration ball_time_to_position = Duration::fromSeconds(
            distance(intercept_position, ball.position()) / ball.velocity().length());
        Duration robot_time_to_pos = getTimeToPositionForRobot(
            robot.position(), intercept_position,
            robot.robotConstants().robot_max_speed_m_per_s,
            robot.robotConstants().robot_max_acceleration_m_per_s_2);

        if (robot_time_to_pos < ball_time_to_position)
        {
            break;
        }
        intercept_position +=
            ball.velocity().normalize(INTERCEPT_POSITION_SEARCH_INTERVAL);
    }
    return intercept_position;
}

Point DribbleFSM::getDribbleBallDestination(const Point &ball_position,
                                            std::optional<Point> dribble_destination)
{
    // Default is the current ball position
    Point target_dest = ball_position;
    if (dribble_destination)
    {
        target_dest = dribble_destination.value();
    }
    return target_dest;
}

Angle DribbleFSM::getFinalDribbleOrientation(
    const Point &ball_position, const Point &robot_position,
    std::optional<Angle> final_dribble_orientation)
{
    // Default is face ball direction
    Angle target_orientation = (ball_position - robot_position).orientation();
    if (final_dribble_orientation)
    {
        target_orientation = final_dribble_orientation.value();
    }
    return target_orientation;
}

std::tuple<Point, Angle> DribbleFSM::calculateNextDribbleDestinationAndOrientation(
    const Ball &ball, const Robot &robot, std::optional<Point> dribble_destination_opt,
    std::optional<Angle> final_dribble_orientation_opt)
{
    Point dribble_destination =
        getDribbleBallDestination(ball.position(), dribble_destination_opt);

    // Default destination and orientation assume ball is at the destination
    // pivot to final face ball destination
    Angle target_orientation = getFinalDribbleOrientation(
        ball.position(), robot.position(), final_dribble_orientation_opt);
    Point target_destination =
        robotPositionToFaceBall(dribble_destination, target_orientation);

    return std::make_tuple(target_destination, target_orientation);
}

void DribbleFSM::getPossession(const Update &event)
{
    auto ball_position = event.common.world.ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();
    Point intercept_position = findInterceptionPoint(
        event.common.robot, event.common.world.ball(), event.common.world.field());
    event.common.set_intent(std::make_unique<MoveIntent>(
        event.common.robot.id(), intercept_position, face_ball_orientation, 0,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants()));
}

void DribbleFSM::dribble(const Update &event)
{
    Point ball_position = event.common.world.ball().position();
    auto [target_destination, target_orientation] =
        calculateNextDribbleDestinationAndOrientation(
            event.common.world.ball(), event.common.robot,
            event.control_params.dribble_destination,
            event.control_params.final_dribble_orientation);
    AutoChipOrKick auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::OFF, 0};

    if (!event.control_params.allow_excessive_dribbling &&
        !comparePoints(ball_position, continuous_dribbling_start_point,
                       MAX_CONTINUOUS_DRIBBLING_DISTANCE))
    {
        // give the ball a little kick
        auto_chip_or_kick =
            AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, DRIBBLE_KICK_SPEED};
    }

    event.common.set_intent(std::make_unique<MoveIntent>(
        event.common.robot.id(), target_destination, target_orientation, 0,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        auto_chip_or_kick, TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants()));
}

void DribbleFSM::startDribble(const Update &event)
{
    // update continuous_dribbling_start_point once we start dribbling
    continuous_dribbling_start_point = event.common.world.ball().position();
    dribble(event);
}

bool DribbleFSM::havePossession(const Update &event)
{
    return event.common.robot.isNearDribbler(event.common.world.ball().position());
}

bool DribbleFSM::dribblingDone(const Update &event)
{
    return comparePoints(
               event.common.world.ball().position(),
               getDribbleBallDestination(event.common.world.ball().position(),
                                         event.control_params.dribble_destination),
               BALL_CLOSE_TO_DEST_THRESHOLD) &&
           compareAngles(
               event.common.robot.orientation(),
               getFinalDribbleOrientation(event.common.world.ball().position(),
                                          event.common.robot.position(),
                                          event.control_params.final_dribble_orientation),
               FINAL_DESTINATION_CLOSE_THRESHOLD) &&
           havePossession(event) &&
           robotStopped(event.common.robot, ROBOT_DRIBBLING_DONE_SPEED);
}
