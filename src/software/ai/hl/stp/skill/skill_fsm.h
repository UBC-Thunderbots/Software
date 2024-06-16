#pragma once

#include "proto/primitive/primitive_msg_factory.h"
#include "proto/primitive/primitive_types.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/hl/stp/primitive/primitive.h"
#include "software/ai/hl/stp/primitive/stop_primitive.h"
#include "software/ai/strategy.h"
#include "software/util/sml_fsm/sml_fsm.h"

using SetPrimitiveCallback = std::function<void(std::shared_ptr<Primitive>)>;

struct SkillUpdate
{
    SkillUpdate(const Robot &robot, const WorldPtr &world_ptr,
                std::shared_ptr<Strategy> strategy,
                const SetPrimitiveCallback &set_primitive_fun)
        : robot(robot),
          world_ptr(world_ptr),
          strategy(strategy),
          set_primitive(set_primitive_fun)
    {
    }
    Robot robot;
    WorldPtr world_ptr;
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

#define DEFINE_SUSPENDED_STATE_AND_UPDATE_STRUCT                                         \
    struct Suspended;                                                                    \
    struct SuspendedUpdate                                                               \
    {                                                                                    \
        SuspendedUpdate(const WorldPtr &world_ptr, std::shared_ptr<Strategy> strategy)   \
            : world_ptr(world_ptr), strategy(strategy)                                   \
        {                                                                                \
        }                                                                                \
        WorldPtr world_ptr;                                                              \
        std::shared_ptr<Strategy> strategy;                                              \
    };

#define SET_STOP_PRIMITIVE_ACTION                                                        \
    [this](auto event) { event.common.set_primitive(std::make_unique<StopPrimitive>()); }
