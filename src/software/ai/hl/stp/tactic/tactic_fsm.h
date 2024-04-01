#pragma once

#include <functional>

#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/hl/stp/primitive/primitive.h"
#include "software/ai/hl/stp/primitive/stop_primitive.h"
#include "software/util/sml_fsm/sml_fsm.h"
#include "software/world/world.h"

using SetPrimitiveCallback = std::function<void(std::shared_ptr<Primitive>)>;

// The tactic update struct is used to update tactics and set the new primitive
struct TacticUpdate
{
    TacticUpdate(const Robot &robot, const WorldPtr &world_ptr,
                 const SetPrimitiveCallback &set_primitive_fun)
        : robot(robot), world_ptr(world_ptr), set_primitive(set_primitive_fun)
    {
    }

    // updated robot that tactic is assigned to
    Robot robot;
    // updated world
    WorldPtr world_ptr;
    // callback to return the next primitive
    SetPrimitiveCallback set_primitive;
};

/**
 * The Update struct is the only event that a tactic fsm should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each tactic to control the FSM
 * TacticUpdate - common struct that contains Robot, World, and SetPrimitiveCallback
 */
#define DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS                       \
    struct Update                                                                        \
    {                                                                                    \
        Update(const ControlParams &control_params, const TacticUpdate &common)          \
            : control_params(control_params), common(common)                             \
        {                                                                                \
        }                                                                                \
        ControlParams control_params;                                                    \
        TacticUpdate common;                                                             \
    };

#define DEFINE_TACTIC_DONE_AND_GET_FSM_STATE                                             \
    bool done() const override                                                           \
    {                                                                                    \
        bool is_done = false;                                                            \
        if (last_execution_robot.has_value())                                            \
        {                                                                                \
            is_done = fsm_map.at(last_execution_robot.value())->is(boost::sml::X);       \
        }                                                                                \
        return is_done;                                                                  \
    }                                                                                    \
                                                                                         \
    std::string getFSMState() const override                                             \
    {                                                                                    \
        std::string state_str = "";                                                      \
        if (last_execution_robot.has_value())                                            \
            state_str =                                                                  \
                getCurrentFullStateName(*fsm_map.at(last_execution_robot.value()));      \
        return state_str;                                                                \
    }

#define SET_STOP_PRIMITIVE_ACTION                                                        \
    [this](auto event) { event.common.set_primitive(std::make_unique<StopPrimitive>()); }
