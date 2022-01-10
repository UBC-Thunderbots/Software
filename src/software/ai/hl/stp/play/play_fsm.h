#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/util/sml_fsm/sml_fsm.h"
#include "software/world/world.h"

using TacticVector              = std::vector<std::shared_ptr<Tactic>>;
using PriorityTacticVector      = std::vector<TacticVector>;
using ConstTacticVector         = std::vector<std::shared_ptr<const Tactic>>;
using ConstPriorityTacticVector = std::vector<ConstTacticVector>;

// This callback is used to return tactics from the fsm
using SetTacticsCallback = std::function<void(PriorityTacticVector)>;

// The play update struct is used to update plays and set the new tactics
struct PlayUpdate
{
    PlayUpdate(const World& world, unsigned int num_tactics,
               const SetTacticsCallback& set_tactics_fun)
        : world(world), num_tactics(num_tactics), set_tactics(set_tactics_fun)
    {
    }
    // updated world
    World world;
    // Number of tactics to set
    unsigned int num_tactics;
    // callback to return the next tactics
    SetTacticsCallback set_tactics;
};

/**
 * The Update struct is the only event that a play fsm should respond to and it is
 * composed of the following structs:
 *
 * ControlParams - uniquely defined by each play to control the FSM
 * PlayUpdate - common struct that contains World and SetTacticsCallback
 */
#define DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS                         \
    struct Update                                                                        \
    {                                                                                    \
        Update(const ControlParams& control_params, const PlayUpdate& common)            \
            : control_params(control_params), common(common)                             \
        {                                                                                \
        }                                                                                \
        ControlParams control_params;                                                    \
        PlayUpdate common;                                                               \
    };
