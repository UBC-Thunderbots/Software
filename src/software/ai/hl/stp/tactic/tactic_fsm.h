#pragma once

#include <functional>

#include "software/ai/intent/intent.h"
#include "software/util/sml_fsm/sml_fsm.h"
#include "software/world/world.h"

// This callback is used to return an intent from the fsm
using SetIntentCallback = std::function<void(std::unique_ptr<Intent>)>;

// The tactic update struct is used to update tactics and set the new intent
struct TacticUpdate
{
    TacticUpdate(const Robot &robot, const World &world,
                 const SetIntentCallback &set_intent_fun)
        : robot(robot), world(world), set_intent(set_intent_fun)
    {
    }
    // updated robot that tactic is assigned to
    Robot robot;
    // updated world
    World world;
    // callback to return the next intent
    SetIntentCallback set_intent;
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


// fsm.visit_current_states([&state_str](auto state) {state_str+=
// std::string(state.c_str());});

// return stripFSMState(state_str);
// GoalieTacticboost::ext::sml::v1_1_3::aux::string<GoalieFSM::PositionToBlockState>
// MoveTacticboost::ext::sml::v1_1_3::aux::string<boost::ext::sml::v1_1_3::back::terminate_state>
// AttackerTacticboost::ext::sml::v1_1_3::aux::string<boost::ext::sml::v1_1_3::back::sm<boost::ext::sml::v1_1_3::back::sm_policy<DribbleFSM>
// >>
// 2- AttackerTacticsm_policy<DribbleFSM
//
// fsm.visit_current_states([&state_str](auto state) {state_str+=
// std::string(TYPENAME(state));});
//
// c_str:
// 1- ShadowEnemyTacticShadowEnemyFSM::BlockPassState
// 2-
// AttackerTacticboost::ext::sml::v1_1_3::back::sm<boost::ext::sml::v1_1_3::back::sm_policy<DribbleFSM>
// >
