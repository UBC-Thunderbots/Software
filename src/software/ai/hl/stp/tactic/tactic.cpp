#include "software/ai/hl/stp/tactic/tactic.h"

#include "software/ai/intent/stop_intent.h"
#include "software/logger/logger.h"
#include "software/util/typename/typename.h"

Tactic::Tactic(const std::set<RobotCapability> &capability_reqs_)
    : last_execution_robot(std::nullopt), intent(), capability_reqs(capability_reqs_)
{
}

const std::set<RobotCapability> &Tactic::robotCapabilityRequirements() const
{
    return capability_reqs;
}

std::set<RobotCapability> &Tactic::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}

std::unique_ptr<Intent> Tactic::get(const Robot &robot, const World &world)
{
    updateIntent(TacticUpdate(robot, world, [this](std::unique_ptr<Intent> new_intent) {
        intent = std::move(new_intent);
    }));

    if (intent)
    {
        return std::move(intent);
    }
    else
    {
        return std::make_unique<StopIntent>(robot.id(), false);
    }
}

void Tactic::setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
{
    this->last_execution_robot = last_execution_robot;
}

std::unique_ptr<TbotsProto::PrimitiveSet> Tactic::get(
    const World &world, std::shared_ptr<const PathPlanner> path_planner)
{
    auto primitive_set = std::make_unique<TbotsProto::PrimitiveSet>();
    for (const auto &robot : world.friendlyTeam().getAllRobots())
    {
        primitive.reset();
        updatePrimitive(
            TacticUpdate(
                robot, world,
                [this](std::unique_ptr<TbotsProto::Primitive> new_primitive) {
                    primitive = std::move(new_primitive);
                },
                path_planner),
            !last_execution_robot.has_value() || last_execution_robot != robot.id());

        if (primitive)
        {
            primitive_set->mutable_robot_primitives()->insert(
                google::protobuf::MapPair(robot.id(), *primitive));
        }
    }
    return primitive_set;
}

void Tactic::updatePrimitive(const TacticUpdate &, bool) {}
