#include "software/ai/hl/stp/tactic/tactic.h"

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/logger/logger.h"
#include "software/util/typename/typename.h"

Tactic::Tactic(const std::set<RobotCapability> &capability_reqs_)
    : last_execution_robot(std::nullopt), capability_reqs(capability_reqs_)
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

void Tactic::setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
{
    this->last_execution_robot = last_execution_robot;
}

std::map<RobotId, std::shared_ptr<Primitive>> Tactic::get(const World &world)
{
    TbotsProto::RobotNavigationObstacleConfig obstacle_config;
    std::map<RobotId, std::shared_ptr<Primitive>> primitives_map;
    for (const auto &robot : world.friendlyTeam().getAllRobots())
    {
        updatePrimitive(TacticUpdate(robot, world,
                                     [this](std::shared_ptr<Primitive> new_primitive) {
                                         primitive = std::move(new_primitive);
                                     }),
                        !last_execution_robot.has_value() ||
                            last_execution_robot.value() != robot.id());

        CHECK(primitive != nullptr)
            << "Primitive for " << objectTypeName(*this) << " in state " << getFSMState()
            << " was not set" << std::endl;
        primitives_map[robot.id()] = std::move(primitive);
    }
    return primitives_map;
}
