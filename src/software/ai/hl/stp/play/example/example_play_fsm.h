#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

/**
 * An example Play that moves the robots in a circle around the ball
 * */

struct ExamplePlayFSM
{
    struct ControlParams
    {
        // TODO
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_EVENT(Update)
        // TODO

        return make_transition_table(
                // src_state + event [guard] / action = dest_state

                ); // TODO
    }
};