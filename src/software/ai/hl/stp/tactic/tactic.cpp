#include "software/ai/hl/stp/tactic/tactic.h"

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

std::unique_ptr<TbotsProto::PrimitiveSet> Tactic::get(
    const World &world, CreateMotionControl create_motion_control)
{
    auto primitive_set = std::make_unique<TbotsProto::PrimitiveSet>();
    for (const auto &robot : world.friendlyTeam().getAllRobots())
    {
        primitive.reset();
        updatePrimitive(TacticUpdate(
                            robot, world,
                            [this](std::unique_ptr<TbotsProto::Primitive> new_primitive) {
                                primitive = std::move(new_primitive);
                            },
                            create_motion_control),
                        !last_execution_robot.has_value() ||
                            last_execution_robot.value() != robot.id());

        if (primitive)
        {
            primitive_set->mutable_robot_primitives()->insert(
                google::protobuf::MapPair(robot.id(), *primitive));
        }
        else
        {
            // Defaulting to setting a stop primitive with cost 0;
            // triggered if the tactic fsm action doesn't set a primitive or if the update
            // doesn't match any transition in the tactic fsm
            primitive_set->mutable_robot_primitives()->insert(
                google::protobuf::MapPair(robot.id(), *createStopPrimitive(false, 0.0)));
        }
    }
    return primitive_set;
}
