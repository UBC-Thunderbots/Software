#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(TbotsProto::AiConfig ai_config,
                               std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      ai_config(ai_config),
      strategy(strategy),
      skill_graph_(strategy),
      skill_map_(),
      skill_fsm_map_()
{
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void AttackerTactic::updateControlParams(const Pass& best_pass_so_far,
                                         bool pass_committed)
{
}

void AttackerTactic::updateControlParams(std::optional<Point> chip_target) {}

std::string AttackerTactic::getFSMState() const
{
    return "AttackerTactic";
}

void AttackerTactic::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    this->ai_config = ai_config;
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    RobotId robot_id = tactic_update.robot.id();

    if (reset_fsm || skill_fsm_map_[robot_id]->done())
    {
        std::shared_ptr<Skill> next_skill =
            skill_graph_.getNextSkill(tactic_update.robot, tactic_update.world);

        if (!reset_fsm && skill_fsm_map_[robot_id]->done())
        {
            skill_graph_.extendSequence(next_skill);
        }

        skill_map_[robot_id] = next_skill;
        skill_fsm_map_[robot_id] = next_skill->getFSM();
    }

    skill_fsm_map_[robot_id]->updatePrimitive(tactic_update);
}
