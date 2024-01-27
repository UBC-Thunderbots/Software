#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(TbotsProto::AiConfig ai_config,
                               std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      ai_config(ai_config),
      strategy(strategy),
      head_skill(std::make_shared<HeadSkill>(ai_config, strategy)),
      skill_sequence({head_skill})
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
    std::string state_str = "";

    if (skill_sequence.size() > 1)
    {
        state_str = skill_sequence.top()->getCurrentState();
    }

    return state_str;
}

void AttackerTactic::setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
{
    if (last_execution_robot &&
        skill_sequence.top() != next_skill_map[last_execution_robot.value()])
    {
        skill_sequence.push(next_skill_map[last_execution_robot.value()]);
    }

    Tactic::setLastExecutionRobot(last_execution_robot);
}

void AttackerTactic::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    this->ai_config = ai_config;
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    std::shared_ptr<Skill> robot_skill = skill_sequence.top();

    auto primitive_set = std::make_unique<TbotsProto::PrimitiveSet>();
    if (reset_fsm || robot_skill->done())
    {
        robot_skill = robot_skill->getNextSkill(tactic_update.robot, tactic_update.world);
    }

    robot_skill->updatePrimitive(tactic_update);
    next_skill_map[tactic_update.robot.id()] = robot_skill;
}
