#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/util/sml_fsm/sml_fsm.h"
#include "software/world/world.h"

using TacticVector              = std::vector<std::shared_ptr<Tactic>>;
using PriorityTacticVector      = std::vector<TacticVector>;
using ConstTacticVector         = std::vector<std::shared_ptr<const Tactic>>;
using ConstPriorityTacticVector = std::vector<ConstTacticVector>;

// Struct used to communicate between plays
struct InterPlayCommunication
{
    std::optional<PassWithRating> last_committed_pass;
};

// This callback is used to return tactics from the fsm
using SetTacticsCallback                = std::function<void(PriorityTacticVector)>;
using SetInterPlayCommunicationCallback = std::function<void(InterPlayCommunication)>;

// The play update struct is used to update plays and set the new tactics
struct PlayUpdate
{
    PlayUpdate(const WorldPtr& world_ptr, unsigned int num_tactics,
               const SetTacticsCallback& set_tactics_fun,
               const InterPlayCommunication& inter_play_communication,
               const SetInterPlayCommunicationCallback& set_inter_play_communication_fun)
        : world_ptr(world_ptr),
          num_tactics(num_tactics),
          set_tactics(set_tactics_fun),
          inter_play_communication(inter_play_communication),
          set_inter_play_communication_fun(set_inter_play_communication_fun)
    {
    }
    // updated world
    WorldPtr world_ptr;
    // Number of tactics to set
    unsigned int num_tactics;
    // callback to return the next tactics
    SetTacticsCallback set_tactics;
    // inter-play communication
    InterPlayCommunication inter_play_communication;
    // callback to return inter-play communication
    SetInterPlayCommunicationCallback set_inter_play_communication_fun;
};

/**
 * The Update struct is the only event that a play FSM should respond to and it is
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
