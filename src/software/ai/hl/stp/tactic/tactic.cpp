#include "software/ai/hl/stp/tactic/tactic.h"

#include <Tracy.hpp>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/logger/logger.h"
#include "software/util/typename/typename.h"
template<class Tactic_FSM>
Tactic<Tactic_FSM>::Tactic(const std::set<RobotCapability> &capability_reqs_, std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : last_execution_robot(std::nullopt), capability_reqs(capability_reqs_), ai_config_ptr(ai_config_ptr), fsm_map()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<Tactic_FSM>>();
    }
}

template<class Tactic_FSM>
bool Tactic<Tactic_FSM>::done() const
    {
        bool is_done = false;
        if (last_execution_robot.has_value())
        {
            is_done = fsm_map.at(last_execution_robot.value())->is(boost::sml::X);
        }
        return is_done;
    }

template<class Tactic_FSM>
std::string Tactic<Tactic_FSM>::getFSMState() const
{
    std::string state_str = "";
    if (last_execution_robot.has_value())
        state_str =
            getCurrentFullStateName(*fsm_map.at(last_execution_robot.value()));
    return state_str;
}

template<class Tactic_FSM>
const std::set<RobotCapability> &Tactic<Tactic_FSM>::robotCapabilityRequirements() const
{
    return capability_reqs;
}

template<class Tactic_FSM>
std::set<RobotCapability> &Tactic<Tactic_FSM>::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}

template<class Tactic_FSM>
void Tactic<Tactic_FSM>::setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
{
    this->last_execution_robot = last_execution_robot;
}

template<class Tactic_FSM>
std::map<RobotId, std::shared_ptr<Primitive>> Tactic<Tactic_FSM>::get(const WorldPtr &world_ptr)
{
    TbotsProto::RobotNavigationObstacleConfig obstacle_config;
    std::map<RobotId, std::shared_ptr<Primitive>> primitives_map;

    {
        ZoneNamedN(_tracy_tactic_set_primitive, "Tactic: Get primitives for each robot",
                   true);

        for (const auto &robot : world_ptr->friendlyTeam().getAllRobots())
        {
            updatePrimitive(TacticUpdate(robot, world_ptr,
                                         [this](std::shared_ptr<Primitive> new_primitive)
                                         { primitive = std::move(new_primitive); }),
                            !last_execution_robot.has_value() ||
                                last_execution_robot.value() != robot.id());

            CHECK(primitive != nullptr)
                << "Primitive for " << objectTypeName(*this) << " in state "
                << getFSMState() << " was not set" << std::endl;
            primitives_map[robot.id()] = std::move(primitive);
        }
    }

    return primitives_map;
}
