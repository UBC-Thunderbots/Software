#include "software/ai/hl/stp/tactic/tactic.h"

#include <Tracy.hpp>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/logger/logger.h"
#include "software/util/typename/typename.h"
template<class TacticFsm, class... TacticSubFsms>
Tactic<TacticFsm, TacticSubFsms...>::Tactic(const std::set<RobotCapability> &capability_reqs_, std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : last_execution_robot(std::nullopt), capability_reqs(capability_reqs_), ai_config_ptr(ai_config_ptr), fsm_map(), control_params()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = fsmInit();
    }
}

template<class TacticFsm, class... TacticSubFsms>
std::unique_ptr<FSM<TacticFsm>> Tactic<TacticFsm, TacticSubFsms...>::fsmInit() {
    return std::make_unique<FSM<TacticFsm>>(TacticFsm(ai_config_ptr), TacticSubFsms(ai_config_ptr)...);
}

template<class TacticFsm, class... TacticSubFsms>
bool Tactic<TacticFsm, TacticSubFsms...>::done() const
    {
        bool is_done = false;
        if (last_execution_robot.has_value())
        {
            is_done = fsm_map.at(last_execution_robot.value())->is(boost::sml::X);
        }
        return is_done;
    }

template<class TacticFsm, class... TacticSubFsms>
std::string Tactic<TacticFsm, TacticSubFsms...>::getFSMState() const
{
    std::string state_str = "";
    if (last_execution_robot.has_value())
        state_str =
            getCurrentFullStateName(*fsm_map.at(last_execution_robot.value()));
    return state_str;
}

template<class TacticFsm, class... TacticSubFsms>
const std::set<RobotCapability> &Tactic<TacticFsm, TacticSubFsms...>::robotCapabilityRequirements() const
{
    return capability_reqs;
}

template<class TacticFsm, class... TacticSubFsms>
std::set<RobotCapability> &Tactic<TacticFsm, TacticSubFsms...>::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}

template<class TacticFsm, class... TacticSubFsms>
void Tactic<TacticFsm, TacticSubFsms...>::setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
{
    this->last_execution_robot = last_execution_robot;
}

template<class TacticFsm, class... TacticSubFsms>
std::map<RobotId, std::shared_ptr<Primitive>> Tactic<TacticFsm, TacticSubFsms...>::get(const WorldPtr &world_ptr)
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

template<class TacticFsm, class... TacticSubFsms>
void Tactic<TacticFsm, TacticSubFsms...>::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsmInit();
    }
    fsm_map.at(tactic_update.robot.id())
            ->process_event(TacticFsm::Update(control_params, tactic_update));
}
