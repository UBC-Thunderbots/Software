#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      strategy(strategy),
      last_execution_robot_changed_(false),
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

void AttackerTactic::setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
{
    if (this->last_execution_robot != last_execution_robot)
    {
        last_execution_robot_changed_ = true;
    }
    Tactic::setLastExecutionRobot(last_execution_robot);
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    std::shared_ptr<Skill> next_skill =
        skill_graph_.getNextSkill(tactic_update.robot, tactic_update.world);

    if (last_execution_robot == tactic_update.robot.id())
    {
        if (current_skill_ == nullptr || last_execution_robot_changed_ ||
            current_skill_->done(tactic_update.robot))
        {
            last_execution_robot_changed_ = false;

            current_skill_ = next_skill;
            current_skill_->reset(tactic_update.robot);
            skill_graph_.extendSequence(current_skill_);
        }

        current_skill_->updatePrimitive(tactic_update.robot, tactic_update.world,
                                        tactic_update.set_primitive);
    }
    else
    {
        next_skill->reset(tactic_update.robot);
        next_skill->updatePrimitive(tactic_update.robot, tactic_update.world,
                                    tactic_update.set_primitive);
    }
}
