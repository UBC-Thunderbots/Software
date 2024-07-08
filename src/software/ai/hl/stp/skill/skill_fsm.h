#pragma once

#include "proto/primitive/primitive_msg_factory.h"
#include "proto/primitive/primitive_types.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/hl/stp/primitive/primitive.h"
#include "software/ai/hl/stp/primitive/stop_primitive.h"
#include "software/ai/hl/stp/skill/skill_state.h"
#include "software/ai/strategy.h"
#include "software/util/sml_fsm/sml_fsm.h"

using SetPrimitiveCallback  = std::function<void(std::shared_ptr<Primitive>)>;
using SetSkillStateCallback = std::function<void(const SkillState &)>;

/**
 * SkillUpdate is a common struct passed to Skill FSMs when they receive an Update event.
 * It contains information about the robot executing the Skill and the current state
 * of the World.
 *
 * Skill FSMs **MUST** respond to an Update event by returning a Primitive describing
 * the action the robot should take. This is done by calling the SkillUpdate's
 * SetPrimitiveCallback with the Primitive to return.
 *
 * Skill FSMs can optionally return information about their current internal state
 * by calling the SkillUpdate's SetSkillStateCallback with a SkillState struct.
 */
struct SkillUpdate
{
    /**
     * Creates a SkillUpdate struct. 
     * 
     * @param robot the robot executing the Skill
     * @param world_ptr the current World
     * @param strategy the Strategy shared by all of AI
     * @param set_primitive_fun callback used by Skill FSM to return a Primitive
     * @param set_skill_state_fun callback used by Skill FSM to return its SkillState
     */
    SkillUpdate(
        const Robot &robot, const WorldPtr &world_ptr, std::shared_ptr<Strategy> strategy,
        const SetPrimitiveCallback &set_primitive_fun,
        const SetSkillStateCallback &set_skill_state_fun =
            [](const SkillState &skill_state) {})
        : robot(robot),
          world_ptr(world_ptr),
          strategy(strategy),
          set_primitive(set_primitive_fun),
          set_skill_state(set_skill_state_fun)
    {
    }
    Robot robot;
    WorldPtr world_ptr;
    std::shared_ptr<Strategy> strategy;
    SetPrimitiveCallback set_primitive;
    SetSkillStateCallback set_skill_state;
};

/**
 * Place this macro in a Skill FSM class definition to define an Update struct. 
 * 
 * The Update struct is the main event that a Skill FSM should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each Skill FSM to control the FSM
 * SkillUpdate   - common struct that contains Robot, World, Strategy,
 *                 SetPrimitiveCallback, and SetSkillStateCallback
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

/**
 * Optionally place this macro in a Skill FSM class definition to define a Suspended 
 * state struct and a SuspendedUpdate event struct.
 * 
 * A Skill FSM can temporarily "suspend" its execution by entering the Suspended state.
 * When a Skill FSM is the Suspended state, it should only respond to the SuspendedUpdate 
 * event and NOT respond to the Update event.
 * 
 * The Skill FSM does NOT return Primitives in the Suspended state.
 * It should only evaluate the state of the World via FSM guards and wait until the 
 * World is in the desired state for the Skill. Once the World is in the desired state,
 * the Skill FSM should leave the Suspended state and return to normal execution.
 *  
 * See Skill::suspended and Skill::tryResumingIfSuspended for more details.
 */
#define DEFINE_SUSPENDED_STATE_AND_UPDATE_STRUCT                                         \
    struct Suspended;                                                                    \
    struct SuspendedUpdate                                                               \
    {                                                                                    \
        SuspendedUpdate(const WorldPtr &world_ptr, std::shared_ptr<Strategy> strategy,   \
                        const SetSkillStateCallback &set_skill_state_fun)                \
            : world_ptr(world_ptr),                                                      \
              strategy(strategy),                                                        \
              set_skill_state(set_skill_state_fun)                                       \
        {                                                                                \
        }                                                                                \
        WorldPtr world_ptr;                                                              \
        std::shared_ptr<Strategy> strategy;                                              \
        SetSkillStateCallback set_skill_state;                                           \
    };

/**
 * Defines a FSM action that calls the given event's SetPrimitiveCallback
 * with StopPrimitive.
 */
#define SET_STOP_PRIMITIVE_ACTION                                                        \
    [this](Update event)                                                                 \
    { event.common.set_primitive(std::make_unique<StopPrimitive>()); }
