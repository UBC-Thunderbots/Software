#pragma once

#include <functional>

#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/navigator/path_planner/enlsvg_path_planner.h"
#include "software/util/sml_fsm/sml_fsm.h"
#include "software/world/world.h"

using SetPrimitiveCallback = std::function<void(std::unique_ptr<TbotsProto::Primitive>)>;
using CreateMotionControl =
    std::function<TbotsProto::MotionControl(const Robot &, const Point &)>;

// The tactic update struct is used to update tactics and set the new primitive
struct TacticUpdate
{
    TacticUpdate(const Robot &robot, const World &world,
                 const SetPrimitiveCallback &set_primitive_fun,
                 const CreateMotionControl &create_motion_control)
        : robot(robot),
          world(world),
          set_primitive(set_primitive_fun),
          create_motion_control(create_motion_control)
    {
    }

    // updated robot that tactic is assigned to
    Robot robot;
    // updated world
    World world;
    // callback to return the next primitive
    SetPrimitiveCallback set_primitive;
    // creator for motion control
    CreateMotionControl create_motion_control;
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

#define CREATE_MOTION_CONTROL(DESTINATION)                                               \
    event.common.create_motion_control(event.common.robot, DESTINATION)

#define SET_STOP_PRIMITIVE_ACTION                                                        \
    [this](auto event) { event.common.set_primitive(createStopPrimitive(false)); }
