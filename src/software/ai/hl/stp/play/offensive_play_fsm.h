#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/keep_away.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/passing/pass.h"

// This callback is used to return an intent from the fsm
using SetTacticsCallback = std::function<void(TacticVector)>;

// The tactic update struct is used to update tactics and set the new intent
struct PlayUpdate
{
    PlayUpdate(const World &world,
                 const SetTacticsCallback &set_tactics_fun)
        : world(world), set_tactics(set_tactics_fun)
    {
    }
    // updated world
    World world;
    // callback to return the next tactics
    SetTacticsCallback set_tactics;
};

/**
 * The Update struct is the only event that a tactic fsm should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each tactic to control the FSM
 * TacticUpdate - common struct that contains World and SetTacticsCallback
 */
#define DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS                              \
    struct Update                                                                        \
    {                                                                                    \
        Update(const ControlParams &control_params, const PlayUpdate &common)          \
            : control_params(control_params), common(common)                             \
        {                                                                                \
        }                                                                                \
        ControlParams control_params;                                                    \
        PlayUpdate common;                                                             \
    };



struct OffensivePlayFSM
{
    class AttemptToShootWhileLookingForAPassState;
    class TakePassState;

    struct ControlParams
    {
        unsigned int num_additional_attackers;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto look_for_pass_s  = state<AttemptToShootWhileLookingForAPassState>;
        const auto take_pass_s  = state<TakePassState>;
        const auto update_e     = event<Update>;


        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *look_for_pass_s + update_e[pass_found] / take_pass     = take_pass_s,
            look_for_pass_s + update_e[!pass_found] / look_for_pass = look_for_pass_s,
            take_pass_s + update_e[!pass_completed] / take_pass     = take_pass_s,
            take_pass_s + update_e[should_abort] / look_for_pass      = look_for_pass_s,
            take_pass_s + update_e[pass_completed] / take_pass      = X,
            X + update_e / look_for_pass                            = look_for_pass_s);
    }
};
