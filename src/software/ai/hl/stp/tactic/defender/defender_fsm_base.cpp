#include "defender_fsm_base.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/geom/algorithms/closest_point.h"

bool DefenderFSMBase::ballNearbyWithoutThreat(
    const WorldPtr& world_ptr, const Robot& robot,
    const TbotsProto::BallStealMode& ball_steal_mode,
    const TbotsProto::DefenderStealConfig& defender_steal_config)
{
    Point robot_position = robot.position();
    Point ball_position  = world_ptr->ball().position();

    std::optional<Robot> nearest_friendly_to_ball =
        world_ptr->friendlyTeam().getNearestRobot(ball_position);
    std::optional<Robot> nearest_enemy_to_ball =
        world_ptr->enemyTeam().getNearestRobot(robot_position);
    if (ball_steal_mode == TbotsProto::BallStealMode::IGNORE)
    {
        // Do nothing if stealing is disabled
        return false;
    }
    else if (nearest_friendly_to_ball.has_value() &&
             robot.id() != nearest_friendly_to_ball.value().id())
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
                (1.0 - defender_steal_config.max_get_ball_ratio_threshold());
        bool ball_is_within_max_range =
            ball_distance_to_friendly <= defender_steal_config.max_get_ball_radius_m();
        bool ball_is_slow = world_ptr->ball().velocity().length() <=
                            defender_steal_config.max_ball_speed_to_get_m_per_s();

        return ball_is_near_friendly && ball_is_within_max_range && ball_is_slow;
    }
    else
    {
        return true;
    }
}

void DefenderFSMBase::prepareGetPossession(
    const TacticUpdate& tactic_update,
    std::shared_ptr<Strategy> strategy,
    boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    Point ball_position     = tactic_update.world_ptr->ball().position();
    Point enemy_goal_center = tactic_update.world_ptr->field().enemyGoal().centre();
    auto ball_to_net_vector = Vector(enemy_goal_center - ball_position);

    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination       = ball_position,
        .final_dribble_orientation = ball_to_net_vector.orientation(),
        .excessive_dribbling_mode  = TbotsProto::ExcessiveDribblingMode::LOSE_BALL};

    processEvent(DribbleSkillFSM::Update(
        control_params, SkillUpdate(tactic_update.robot, tactic_update.world_ptr, strategy,
                                    tactic_update.set_primitive)));
}
