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
 * The Update struct is the only event that a Skill FSM should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each Skill FSM to control the FSM
 * SkillUpdate - common struct that contains Robot, World, Strategy, and
 * SetPrimitiveCallback
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
