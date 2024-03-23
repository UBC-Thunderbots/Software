#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      strategy(strategy),
      skill_graph_(strategy),
      current_skill_(nullptr)
{
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

std::string AttackerTactic::getFSMState() const
{
    std::string state = "Unknown";
    if (current_skill_ != nullptr && last_execution_robot)
    {
        state = current_skill_->getFSMState(*last_execution_robot);
    }
    return state;
}

bool AttackerTactic::done() const
{
    if (last_execution_robot)
    {
        return current_skill_->done(*last_execution_robot);
    }

    return true;
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    std::shared_ptr<Skill> next_skill =
        skill_graph_.getNextSkill(tactic_update.robot, *tactic_update.world_ptr);

    if (last_execution_robot == tactic_update.robot.id())
    {
        if (current_skill_ == nullptr || current_skill_->done(tactic_update.robot.id()))
        {
            current_skill_ = next_skill;
            current_skill_->reset(tactic_update.robot);
            skill_graph_.extendSequence(current_skill_);
        }

        current_skill_->updatePrimitive(tactic_update.robot, tactic_update.world_ptr,
                                        tactic_update.set_primitive);
    }
    else
    {
        next_skill->reset(tactic_update.robot);
        next_skill->updatePrimitive(tactic_update.robot, tactic_update.world_ptr,
                                    tactic_update.set_primitive);
    }
}

void AttackerTactic::evaluate(double score)
{
    skill_graph_.scoreSequence(score);
    current_skill_ = nullptr;
}
