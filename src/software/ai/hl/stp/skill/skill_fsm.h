#pragma once

#include "software/ai/hl/stp/strategy/strategy.h"

using SetPrimitiveCallback = std::function<void(std::shared_ptr<Primitive>)>;

struct SkillUpdate
{
    SkillUpdate(const Robot &robot, const World &world,
                std::shared_ptr<Strategy> strategy,
                const SetPrimitiveCallback &set_primitive_fun)
        : robot(robot), world(world), strategy(strategy), set_primitive(set_primitive_fun)
    {
    }
    Robot robot;
    World world;
    std::shared_ptr<Strategy> strategy;
    SetPrimitiveCallback set_primitive;
};

/**
 * The Update struct is the only event that a tactic fsm should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each tactic to control the FSM
 * TacticUpdate - common struct that contains Robot, World, and SetPrimitiveCallback
 */
#define DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS                        \
    struct Update                                                                        \
    {                                                                                    \
        Update(const ControlParams &control_params, const SkillUpdate &common)           \
            : control_params(control_params), common(common)                             \
        {                                                                                \
        }                                                                                \
        ControlParams control_params;                                                    \
        SkillUpdate common;                                                              \
    };

#define DEFINE_SKILL_DONE_AND_GET_FSM_STATE                                              \
    bool done(const Robot &robot) const override                                         \
    {                                                                                    \
        return fsm_map_.contains(robot.id()) &&                                          \
               fsm_map_.at(robot.id())->is(boost::sml::X);                               \
    }                                                                                    \
                                                                                         \
    std::string getFSMState(RobotId robot_id) const override                             \
    {                                                                                    \
        std::string state_str = TYPENAME(*this);                                         \
        if (fsm_map_.contains(robot_id))                                                 \
            state_str += "." + getCurrentFullStateName(*fsm_map_.at(robot_id));          \
        return state_str;                                                                \
    }
