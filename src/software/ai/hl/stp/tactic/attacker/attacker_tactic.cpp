#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      strategy(strategy),
      last_execution_robot_changed_(false),
      skill_graph_(strategy),
      skill_map_(),
      skill_fsm_map_()
{
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

std::string AttackerTactic::getFSMState() const
{
    return "AttackerTactic";
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
    RobotId robot_id = tactic_update.robot.id();

    std::shared_ptr<Skill> next_skill =
        skill_graph_.getNextSkill(tactic_update.robot, tactic_update.world);

    if (!skill_map_.contains(robot_id) || last_execution_robot != robot_id)
    {
        skill_map_[robot_id]     = next_skill;
        skill_fsm_map_[robot_id] = next_skill->getFSM();
    }
    else if (last_execution_robot == robot_id)
    {
        if (skill_fsm_map_[robot_id]->done())
        {
            skill_graph_.extendSequence(next_skill);
            skill_map_[robot_id]     = next_skill;
            skill_fsm_map_[robot_id] = next_skill->getFSM();
        }
        else if (last_execution_robot_changed_)
        {
            skill_graph_.extendSequence(skill_map_[robot_id]);
            last_execution_robot_changed_ = false;
        }
    }

    skill_fsm_map_[robot_id]->updatePrimitive(tactic_update);
}
