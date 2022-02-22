#pragma once

#include <functional>

#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/intent/intent.h"
#include "software/ai/navigator/path_planner/path_planner.h"
#include "software/util/sml_fsm/sml_fsm.h"
#include "software/world/world.h"

// This callback is used to return an intent from the fsm
using SetIntentCallback    = std::function<void(std::unique_ptr<Intent>)>;
using SetPrimitiveCallback = std::function<void(std::unique_ptr<TbotsProto::Primitive>)>;

// The tactic update struct is used to update tactics and set the new intent
struct TacticUpdate
{
    // TODO: remove this function
    TacticUpdate(const Robot &robot, const World &world,
                 const SetIntentCallback &set_intent_fun)
        : robot(robot),
          world(world),
          set_intent(set_intent_fun),
          set_primitive([](std::unique_ptr<TbotsProto::Primitive>) {})
    {
    }

    TacticUpdate(const Robot &robot, const World &world,
                 const SetPrimitiveCallback &set_primitive_fun,
                 std::shared_ptr<const PathPlanner> path_planner)
        : robot(robot),
          world(world),
          set_intent([](std::unique_ptr<Intent>) {}),
          set_primitive(set_primitive_fun),
          path_planner(path_planner)
    {
    }

    // updated robot that tactic is assigned to
    Robot robot;
    // updated world
    World world;
    // callback to return the next intent
    SetIntentCallback set_intent;
    SetPrimitiveCallback set_primitive;

    std::shared_ptr<const PathPlanner> path_planner;
};

/**
 * The Update struct is the only event that a tactic fsm should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each tactic to control the FSM
 * TacticUpdate - common struct that contains Robot, World, and SetIntentCallback
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
        return fsm.is(boost::sml::X);                                                    \
    }                                                                                    \
                                                                                         \
    std::string getFSMState() const override                                             \
    {                                                                                    \
        std::string state_str = getCurrentFullStateName(fsm);                            \
        return state_str;                                                                \
    }
