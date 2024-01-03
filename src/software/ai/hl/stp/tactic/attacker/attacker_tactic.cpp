#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      ai_config(ai_config)
{
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void AttackerTactic::setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
{
    Tactic::setLastExecutionRobot(last_execution_robot);
    if (last_execution_robot &&
        skill_sequence.top() != next_skill_map[last_execution_robot.value().id()])
    {
        skill_sequence.push(next_skill_map[last_execution_robot.value().id()]);
    }
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    Skill robot_skill = skill_sequence.top();

    auto primitive_set = std::make_unique<TbotsProto::PrimitiveSet>();
    if (reset_fsm || !robot_skill || robot_skill.isDone())
    {
        primitive.reset();

        robot_skill = calculateNextSkill(tactic_update.robot, tactic_update.world,
                                         tactic_update.strategy);
    }

    next_skill_map[tactic_update.robot.id()] = robot_skill;

    // TODO(arun) process the tactic update obstain a primitive

    return primitive_set;
}
