#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"

Point DribbleFSM::robotPositionToFaceBall(const Point &ball_position,
                                          const Angle &face_ball_angle,
                                          double additional_offset)
{
    return ball_position - Vector::createFromAngle(face_ball_angle)
                               .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                          BALL_MAX_RADIUS_METERS + additional_offset);
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
        Duration robot_time_to_pos = robot.getTimeToPosition(intercept_position);

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
    Point intercept_position =
        findInterceptionPoint(event.common.robot, event.common.world.ball(),
                              event.common.world.field()) +
        Vector::createFromAngle(face_ball_orientation).normalize(0.05);

    // when close to ball, pivot smartly
    double pivot_start_distance = event.control_params.pivot_start_distance.value_or(0.0);

    if (distance(event.common.robot.position(), ball_position) <
        pivot_start_distance)
    {
        // find the target destination and orientation after the dribble
        auto [target_destination, target_orientation] =
            calculateNextDribbleDestinationAndOrientation(
                event.common.world.ball(), event.common.robot,
                event.control_params.dribble_destination,
                event.control_params.final_dribble_orientation);

        int pivot_threshold_deg = 40;  // this should be <= pivot threshold in dribble()
        double pivot_amount_deg = 60;
        if (target_orientation.minDiff(event.common.robot.orientation()) >
            Angle::fromDegrees(pivot_threshold_deg))
        {
            Angle remaining_rotation =
                (target_orientation - event.common.robot.orientation()).clamp();
            double amount_to_rotate_deg = std::clamp(remaining_rotation.toDegrees(),
                                                     -pivot_amount_deg, pivot_amount_deg);
            Angle target_angle          = event.common.robot.orientation() +
                                 Angle::fromDegrees(amount_to_rotate_deg);

            Vector target_vector =
                ROBOT_MAX_RADIUS_METERS * Vector::createFromAngle(target_angle);
            intercept_position -= target_vector;
        }
    }

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(intercept_position), face_ball_orientation, 0,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants()));
}

void DribbleFSM::dribble(const Update &event)
{
    auto [target_destination, target_orientation] =
        calculateNextDribbleDestinationAndOrientation(
            event.common.world.ball(), event.common.robot,
            event.control_params.dribble_destination,
            event.control_params.final_dribble_orientation);

    Point dribble_destination = getDribbleBallDestination(
        event.common.world.ball().position(), event.control_params.dribble_destination);
    double pivot_amount_deg = 40;
    int pivot_threshold_deg = 40;

    Angle remaining_rotation =
        (target_orientation - event.common.robot.orientation()).clamp();
    double amount_to_rotate_deg =
        std::clamp(remaining_rotation.toDegrees(), -pivot_amount_deg, pivot_amount_deg);
    Angle target_angle =
        event.common.robot.orientation() + Angle::fromDegrees(amount_to_rotate_deg);

    if (target_angle.minDiff(target_orientation) <
        Angle::fromDegrees(pivot_threshold_deg))
    {
        target_angle = target_orientation;
    }

    target_destination = dribble_destination - (ROBOT_MAX_RADIUS_METERS *
                                                Vector::createFromAngle(target_angle));
    target_orientation = target_angle;

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(target_destination), target_orientation, 0,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants(), 0.0));
}

void DribbleFSM::loseBall(const Update &event)
{
    Point ball_position = event.common.world.ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();
    Point away_from_ball_position = robotPositionToFaceBall(
        ball_position, face_ball_orientation,
        dribble_tactic_config.lose_ball_possession_threshold() * 2);

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(away_from_ball_position), face_ball_orientation, 0,
        TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, 0.5},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants(), 0.0));
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

bool DribbleFSM::lostPossession(const Update &event)
{
    return !event.common.robot.isNearDribbler(
        // avoid cases where ball is exactly on the edge of the robot
        event.common.world.ball().position(),
        dribble_tactic_config.lose_ball_possession_threshold());
};

bool DribbleFSM::dribblingDone(const Update &event)
{
    return comparePoints(
               event.common.world.ball().position(),
               getDribbleBallDestination(event.common.world.ball().position(),
                                         event.control_params.dribble_destination),
               dribble_tactic_config.ball_close_to_dest_threshold()) &&
           compareAngles(
               event.common.robot.orientation(),
               getFinalDribbleOrientation(event.common.world.ball().position(),
                                          event.common.robot.position(),
                                          event.control_params.final_dribble_orientation),
               Angle::fromDegrees(
                   dribble_tactic_config.final_destination_close_threshold())) &&
           havePossession(event) &&
           robotStopped(event.common.robot,
                        dribble_tactic_config.robot_dribbling_done_speed());
}

bool DribbleFSM::shouldLoseBall(const Update &event)
{
    Point ball_position = event.common.world.ball().position();
    return (!event.control_params.allow_excessive_dribbling &&
            !comparePoints(ball_position, continuous_dribbling_start_point,
                           dribble_tactic_config.max_continuous_dribbling_distance()));
}
