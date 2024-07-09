#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/tactic/move_primitive.h"
#include "software/geom/algorithms/closest_point.h"

bool PassDefenderFSM::passStarted(const Update& event)
{
    auto ball_position = event.common.world_ptr->ball().position();
    Vector ball_receiver_point_vector(
        event.control_params.position_to_block_from.x() - ball_position.x(),
        event.control_params.position_to_block_from.y() - ball_position.y());

    bool pass_started = event.common.world_ptr->ball().hasBallBeenKicked(
        ball_receiver_point_vector.orientation(), MIN_PASS_SPEED,
        MAX_PASS_ANGLE_DIFFERENCE);

    if (pass_started)
    {
        // We want to keep track of the initial trajectory of the pass
        // so that we can later tell whether the ball strays from
        // this trajectory
        pass_orientation = event.common.world_ptr->ball().velocity().orientation();
    }

    return pass_started;
}

bool PassDefenderFSM::ballDeflected(const Update& event)
{
    auto orientation_difference =
        event.common.world_ptr->ball().velocity().orientation().minDiff(pass_orientation);

    // If the ball strays from the initial trajectory of the pass,
    // it was likely deflected off course by another robot or chipped
    // away by the pass defender
    return orientation_difference.abs() > MIN_DEFLECTION_ANGLE;
}

void PassDefenderFSM::blockPass(const Update& event)
{
    auto position_to_block_from = event.control_params.position_to_block_from;
    auto ball_position          = event.common.world_ptr->ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    // Face the ball and move to position_to_block_from, which should be a location
    // on the field that blocks a passing lane between two enemy robots
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, position_to_block_from, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

void PassDefenderFSM::interceptBall(const Update& event)
{
    auto ball           = event.common.world_ptr->ball();
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
        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, intercept_position, face_ball_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
        return;
    }

    // The ball is likely above the robot
    // to avoid dividing by 0
    Angle face_ball_orientation;
    if ((ball.position() - robot_position).length() == 0)
    {
        face_ball_orientation = event.common.robot.orientation();
    }
    else
    {
        face_ball_orientation = (ball.position() - robot_position).orientation();
    }

    // backup by the length of the ball
    Point backup_position =
        robot_position + ball.velocity().normalize(ROBOT_MAX_RADIUS_METERS);
    event.common.set_primitive(std::make_shared<MovePrimitive>(
        event.common.robot, backup_position, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

bool PassDefenderFSM::ballNearbyWithoutThreat(const Update& event)
{
    Point robot_position = event.common.robot.position();
    Point ball_position  = event.common.world_ptr->ball().position();

    std::optional<Robot> nearest_friendly_to_ball =
        event.common.world_ptr->friendlyTeam().getNearestRobot(ball_position);
    std::optional<Robot> nearest_enemy_to_ball =
        event.common.world_ptr->enemyTeam().getNearestRobot(ball_position);
    if (event.control_params.ball_steal_mode == TbotsProto::BallStealMode::IGNORE)
    {
        // Do nothing if stealing is disabled
        return false;
    }
    else if (nearest_friendly_to_ball.has_value() &&
             event.common.robot.id() != nearest_friendly_to_ball.value().id())
    {
        // Do nothing if this robot is not the closest to the ball. Resolves issue of
        // multiple simultaneous steals
        return false;
    }
    if (nearest_enemy_to_ball.has_value())
    {
        // Get the ball if ball is closer to robot than enemy threat by threshold ratio
        // and within max range
        double ball_distance_to_friendly = distance(robot_position, ball_position);
        double ball_distance_to_enemy =
            distance(nearest_enemy_to_ball.value().position(), ball_position);

        bool ball_is_near_friendly =
            ball_distance_to_friendly <
            ball_distance_to_enemy *
                (1.0 - pass_defender_config.max_get_ball_ratio_threshold());
        bool ball_is_within_max_range =
            ball_distance_to_friendly <= pass_defender_config.max_get_ball_radius_m();
        bool ball_is_slow = event.common.world_ptr->ball().velocity().length() <=
                            pass_defender_config.max_ball_speed_to_get_m_per_s();

        return ball_is_near_friendly && ball_is_within_max_range && ball_is_slow;
    }
    else
    {
        return true;
    }
}

void PassDefenderFSM::prepareGetPossession(
    const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent)
{
    Point ball_position     = event.common.world_ptr->ball().position();
    Point enemy_goal_center = event.common.world_ptr->field().enemyGoal().centre();
    auto ball_to_net_vector = Vector(enemy_goal_center - ball_position);
    DribbleFSM::ControlParams control_params{
        .dribble_destination       = ball_position,
        .final_dribble_orientation = ball_to_net_vector.orientation(),
        .allow_excessive_dribbling = false};
    processEvent(DribbleFSM::Update(control_params, event.common));
}
