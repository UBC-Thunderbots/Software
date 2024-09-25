#include "software/ai/hl/stp/play/test_plays/move_test_play.h"

#include "software/ai/hl/stp/skill/move/move_skill.h"
#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"
#include "software/util/generic_factory/generic_factory.h"

MoveTestPlay::MoveTestPlay(const TbotsProto::AiConfig &config,
                           std::shared_ptr<Strategy> strategy)
    : Play(false, strategy)
{
}

void MoveTestPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                  const WorldPtr &world_ptr)
{
    auto move_test_tactic_friendly_goal =
        std::make_shared<AssignedSkillTactic<MoveSkill>>(strategy);
    auto move_test_tactic_enemy_goal =
        std::make_shared<AssignedSkillTactic<MoveSkill>>(strategy);
    auto move_test_tactic_center_field =
        std::make_shared<AssignedSkillTactic<MoveSkill>>(strategy);

    do
    {
        move_test_tactic_friendly_goal->updateControlParams(
            {.destination             = world_ptr->field().friendlyGoalCenter(),
             .final_orientation       = Angle::zero(),
             .final_speed             = 0,
             .max_allowed_speed_mode  = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
             .obstacle_avoidance_mode = TbotsProto::ObstacleAvoidanceMode::SAFE});
        move_test_tactic_enemy_goal->updateControlParams(
            {.destination             = world_ptr->field().enemyGoalCenter(),
             .final_orientation       = Angle::zero(),
             .final_speed             = 0,
             .max_allowed_speed_mode  = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
             .obstacle_avoidance_mode = TbotsProto::ObstacleAvoidanceMode::SAFE});
        move_test_tactic_center_field->updateControlParams(
            {.destination             = world_ptr->field().centerPoint(),
             .final_orientation       = Angle::zero(),
             .final_speed             = 0,
             .max_allowed_speed_mode  = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
             .obstacle_avoidance_mode = TbotsProto::ObstacleAvoidanceMode::SAFE});

        yield({{move_test_tactic_center_field, move_test_tactic_friendly_goal,
                move_test_tactic_enemy_goal}});
    } while (!move_test_tactic_center_field->done());
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, MoveTestPlay, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>>
    factory;
